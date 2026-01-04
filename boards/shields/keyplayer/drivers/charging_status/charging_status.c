/*
 * TP4056 CHRG引脚状态监测驱动 - 低功耗且及时响应版
 * 硬编码使用nRF52840 P1.09引脚
 * 保证状态变化及时检测，同时优化功耗
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include "charging_status.h"
#include "charging_breathe.h"

/* 硬编码日志级别 - 设置为INFO级别，重要事件可见 */
#define CHARGING_STATUS_LOG_LEVEL 3  /* INFO级别 */

LOG_MODULE_REGISTER(charging_status, CHARGING_STATUS_LOG_LEVEL);

/* 硬编码配置 - P1.09引脚 */
#define CHRG_GPIO_PORT DT_NODELABEL(gpio1)  /* GPIO1外设 */
#define CHRG_GPIO_PIN  9                    /* P1.09引脚 */

/* 响应性优化配置 */
#define DEBOUNCE_DELAY_MS 200        /* 防抖延迟200ms（保证响应速度） */
#define MIN_POLL_INTERVAL_SEC 1      /* 最小轮询间隔1秒（充电期间） */
#define NORMAL_POLL_INTERVAL_SEC 5   /* 正常轮询间隔5秒 */
#define MAX_POLL_INTERVAL_SEC 30     /* 最大轮询间隔30秒 */
#define QUICK_POLL_DURATION_SEC 60   /* 快速轮询持续时间60秒（状态变化后） */

/* 状态结构体 */
struct charging_status_data {
    const struct device *gpio_dev;     /* GPIO设备指针 */
    struct gpio_callback gpio_cb;      /* GPIO回调 */
    struct k_work_delayable work;      /* 防抖工作队列 */
    struct k_work_delayable poll_work; /* 轮询工作队列 */
    struct k_work_delayable quick_poll_work; /* 快速轮询结束工作队列 */
    
    charging_state_t current_state;    /* 当前充电状态 */
    charging_state_t last_state;       /* 上一次状态 */
    int raw_pin_state;                 /* 引脚原始电平 */
    int last_raw_pin_state;            /* 上一次原始电平 */
    
    /* 响应性优化相关 */
    uint32_t state_change_count;       /* 状态变化计数 */
    uint32_t interrupt_count;          /* 中断计数 */
    uint32_t poll_interval_sec;        /* 当前轮询间隔（秒） */
    int64_t last_change_time;          /* 上次状态变化时间（毫秒） */
    bool is_quick_polling;             /* 是否处于快速轮询模式 */
    bool is_debouncing;                /* 是否正在防抖 */
    bool interrupt_enabled;            /* 中断是否启用 */
};

/* 全局实例 */
static struct charging_status_data charger_data;

/* 状态字符串映射 */
static const char* state_strings[] = {
    "UNKNOWN",
    "CHARGING",
    "NOT_CHARGING"
};

/* 内部函数声明 */
static void chrg_gpio_callback(const struct device *dev,
                               struct gpio_callback *cb,
                               uint32_t pins);
static void debounce_work_handler(struct k_work *work);
static void poll_work_handler(struct k_work *work);
static void quick_poll_end_handler(struct k_work *work);
static void update_charging_state(int pin_state);
static void report_state_change(charging_state_t old_state, charging_state_t new_state);
static int read_chrg_pin(void);
static void update_poll_interval(void);
static void start_quick_poll_mode(void);
static void end_quick_poll_mode(void);

/* GPIO中断回调函数 - 保持始终启用 */
static void chrg_gpio_callback(const struct device *dev,
                               struct gpio_callback *cb,
                               uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    
    if (pins & BIT(CHRG_GPIO_PIN)) {
        /* 增加中断计数 */
        charger_data.interrupt_count++;
        
        /* 如果已经在防抖中，取消之前的防抖工作，重新开始 */
        if (charger_data.is_debouncing) {
            k_work_cancel_delayable(&charger_data.work);
        } else {
            /* 记录中断发生，用于调试 */
            LOG_DBG("GPIO中断触发 #%u", charger_data.interrupt_count);
        }
        
        /* 标记为防抖中 */
        charger_data.is_debouncing = true;
        
        /* 调度防抖工作（200ms后读取稳定状态） */
        k_work_reschedule(&charger_data.work, K_MSEC(DEBOUNCE_DELAY_MS));
    }
}

/* 防抖工作队列处理函数 */
static void debounce_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    
    /* 读取当前引脚状态 */
    int pin_state = read_chrg_pin();
    
    if (pin_state < 0) {
        LOG_ERR("读取CHRG引脚失败: %d", pin_state);
        charger_data.is_debouncing = false;
        return;
    }
    
    /* 更新状态 */
    update_charging_state(pin_state);
    
    /* 防抖结束 */
    charger_data.is_debouncing = false;
}

/* 轮询工作队列处理函数 */
static void poll_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    
    /* 读取当前引脚状态 */
    int pin_state = read_chrg_pin();
    
    if (pin_state >= 0) {
        /* 更新状态 */
        update_charging_state(pin_state);
    }
    
    /* 更新轮询间隔（如果需要） */
    update_poll_interval();
    
    /* 重新调度轮询工作 */
    k_work_schedule(&charger_data.poll_work, K_SECONDS(charger_data.poll_interval_sec));
    
    /* 调试日志（仅当状态变化频繁时输出） */
    static uint32_t debug_counter = 0;
    if (charger_data.interrupt_count > 50 && (debug_counter++ % 10) == 0) {
        LOG_DBG("轮询: 状态=%s, 中断次数=%u, 间隔=%us",
               state_strings[charger_data.current_state],
               charger_data.interrupt_count,
               charger_data.poll_interval_sec);
    }
}

/* 快速轮询结束处理函数 */
static void quick_poll_end_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    
    /* 结束快速轮询模式 */
    end_quick_poll_mode();
}

/* 读取CHRG引脚电平 */
static int read_chrg_pin(void)
{
    if (!charger_data.gpio_dev) {
        LOG_ERR("GPIO设备未初始化");
        return -ENODEV;
    }
    
    int pin_state = gpio_pin_get(charger_data.gpio_dev, CHRG_GPIO_PIN);
    
    if (pin_state < 0) {
        LOG_ERR("GPIO读取错误: %d", pin_state);
        return pin_state;
    }
    
    return pin_state;
}

/* 更新轮询间隔（智能调整） */
static void update_poll_interval(void)
{
    uint32_t old_interval = charger_data.poll_interval_sec;
    
    if (charger_data.is_quick_polling) {
        /* 快速轮询模式下使用1秒间隔 */
        charger_data.poll_interval_sec = MIN_POLL_INTERVAL_SEC;
        return;
    }
    
    /* 根据当前状态和稳定性调整间隔 */
    if (charger_data.current_state == CHARGING_STATE_CHARGING) {
        /* 正在充电：需要更频繁检测（5秒） */
        charger_data.poll_interval_sec = NORMAL_POLL_INTERVAL_SEC;
    } else {
        /* 未充电：可以延长检测间隔 */
        /* 如果最近1分钟内有状态变化，保持5秒间隔 */
        int64_t now = k_uptime_get();
        int64_t last_change_ms = charger_data.last_change_time;
        
        if ((now - last_change_ms) < (60 * 1000)) { /* 1分钟内 */
            charger_data.poll_interval_sec = NORMAL_POLL_INTERVAL_SEC;
        } else {
            /* 长时间稳定：延长到30秒 */
            charger_data.poll_interval_sec = MAX_POLL_INTERVAL_SEC;
        }
    }
    
    if (old_interval != charger_data.poll_interval_sec) {
        LOG_DBG("轮询间隔调整: %us -> %us", old_interval, charger_data.poll_interval_sec);
    }
}

/* 开始快速轮询模式 */
static void start_quick_poll_mode(void)
{
    if (!charger_data.is_quick_polling) {
        charger_data.is_quick_polling = true;
        LOG_DBG("进入快速轮询模式");
        
        /* 取消现有的轮询工作，立即重新调度 */
        k_work_cancel_delayable(&charger_data.poll_work);
        charger_data.poll_interval_sec = MIN_POLL_INTERVAL_SEC;
        k_work_schedule(&charger_data.poll_work, K_SECONDS(charger_data.poll_interval_sec));
        
        /* 设置60秒后结束快速轮询 */
        k_work_schedule(&charger_data.quick_poll_work, K_SECONDS(QUICK_POLL_DURATION_SEC));
    }
}

/* 结束快速轮询模式 */
static void end_quick_poll_mode(void)
{
    if (charger_data.is_quick_polling) {
        charger_data.is_quick_polling = false;
        LOG_DBG("退出快速轮询模式");
        
        /* 更新到正常轮询间隔 */
        update_poll_interval();
        
        /* 取消现有的轮询工作，使用新间隔重新调度 */
        k_work_cancel_delayable(&charger_data.poll_work);
        k_work_schedule(&charger_data.poll_work, K_SECONDS(charger_data.poll_interval_sec));
    }
}

/* 更新充电状态 */
static void update_charging_state(int pin_state)
{
    charging_state_t new_state;
    
    /* 根据TP4056数据手册判断状态 */
    if (pin_state == 0) {
        new_state = CHARGING_STATE_CHARGING;
    } else {
        new_state = CHARGING_STATE_NOT_CHARGING;
    }
    
    /* 保存原始引脚状态 */
    charger_data.raw_pin_state = pin_state;
    
    /* 检查状态是否真正变化 */
    if (new_state != charger_data.current_state) {
        charging_state_t old_state = charger_data.current_state;
        charger_data.last_state = old_state;
        charger_data.current_state = new_state;
        charger_data.state_change_count++;
        
        /* 记录状态变化时间 */
        charger_data.last_change_time = k_uptime_get();
        
        /* 报告状态变化 */
        report_state_change(old_state, new_state);
        
        /* 状态变化后进入快速轮询模式（确保及时检测后续变化） */
        start_quick_poll_mode();
    } else if (pin_state != charger_data.last_raw_pin_state) {
        /* 电平变化但状态未变（可能是在临界状态抖动） */
        charger_data.last_raw_pin_state = pin_state;
        LOG_DBG("电平抖动: 引脚=%d, 状态=%s", pin_state, state_strings[charger_data.current_state]);
    }
}

/* 报告状态变化 */
static void report_state_change(charging_state_t old_state, charging_state_t new_state)
{
    /* 获取引脚状态 */
    int pin_state = charger_data.raw_pin_state;
    
    /* 输出充电状态变化日志 */
    LOG_INF("========================================");
    LOG_INF("充电状态变化检测到！");
    LOG_INF("========================================");
    LOG_INF("变化时间: %lldms", (long long)k_uptime_get());
    LOG_INF("旧状态: %s", state_strings[old_state]);
    LOG_INF("新状态: %s", state_strings[new_state]);
    LOG_INF("CHRG引脚电平: %d (%s)", 
           pin_state,
           pin_state == 0 ? "低电平" : "高电平");
    LOG_INF("TP4056状态: %s",
           new_state == CHARGING_STATE_CHARGING ? 
           "正在充电" : "未充电/充电完成");
    LOG_INF("状态变化次数: %u", charger_data.state_change_count);
    LOG_INF("中断触发次数: %u", charger_data.interrupt_count);
    LOG_INF("响应延迟: %dms", DEBOUNCE_DELAY_MS);
    LOG_INF("========================================");
    
    /* 更新上次报告的状态 */
    charger_data.last_reported_state = new_state;
    
    /* 通知呼吸灯状态变化 */
    if (new_state == CHARGING_STATE_CHARGING) {
        /* 开始充电：启用呼吸效果 */
        charging_breathe_set_state(true);
    } else {
        /* 停止充电：关闭呼吸效果 */
        charging_breathe_set_state(false);
    }
}

/* 初始化函数 */
static int charging_status_init(void)
{
    int ret;
    
    LOG_INF("初始化TP4056 CHRG引脚监测器");
    LOG_INF("设计目标: 低功耗 + 及时响应");
    LOG_INF("使用硬编码引脚: P1.09 (GPIO1_09)");
    LOG_INF("防抖延迟: %dms", DEBOUNCE_DELAY_MS);
    
    /* 初始化数据结构 */
    charger_data.current_state = CHARGING_STATE_UNKNOWN;
    charger_data.last_state = CHARGING_STATE_UNKNOWN;
    charger_data.raw_pin_state = -1;
    charger_data.last_raw_pin_state = -1;
    charger_data.state_change_count = 0;
    charger_data.interrupt_count = 0;
    charger_data.poll_interval_sec = NORMAL_POLL_INTERVAL_SEC;
    charger_data.is_quick_polling = false;
    charger_data.is_debouncing = false;
    charger_data.interrupt_enabled = false;
    charger_data.last_change_time = 0;
    
    /* 获取GPIO1设备 */
    charger_data.gpio_dev = DEVICE_DT_GET(CHRG_GPIO_PORT);
    if (!device_is_ready(charger_data.gpio_dev)) {
        LOG_ERR("GPIO1设备未就绪");
        return -ENODEV;
    }
    
    LOG_INF("GPIO1设备就绪");
    
    /* 配置P1.09引脚为输入模式，启用内部上拉电阻
     * 重要：使用nRF52840的GPIO中断功能，功耗很低
     */
    ret = gpio_pin_configure(charger_data.gpio_dev,
                            CHRG_GPIO_PIN,
                            GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) {
        LOG_ERR("配置P1.09失败: %d", ret);
        return ret;
    }
    
    LOG_INF("P1.09配置为输入模式，启用上拉电阻");
    
    /* 配置GPIO中断 - 双边沿触发（始终启用） */
    ret = gpio_pin_interrupt_configure(charger_data.gpio_dev,
                                      CHRG_GPIO_PIN,
                                      GPIO_INT_EDGE_BOTH);
    if (ret < 0) {
        LOG_ERR("配置中断失败: %d", ret);
        return ret;
    }
    
    /* 初始化GPIO回调函数 */
    gpio_init_callback(&charger_data.gpio_cb,
                      chrg_gpio_callback,
                      BIT(CHRG_GPIO_PIN));
    
    /* 添加回调函数 */
    ret = gpio_add_callback(charger_data.gpio_dev, &charger_data.gpio_cb);
    if (ret < 0) {
        LOG_ERR("添加GPIO回调失败: %d", ret);
        return ret;
    }
    
    charger_data.interrupt_enabled = true;
    LOG_INF("GPIO中断已启用（始终）");
    
    /* 初始化工作队列 */
    k_work_init_delayable(&charger_data.work, debounce_work_handler);
    k_work_init_delayable(&charger_data.poll_work, poll_work_handler);
    k_work_init_delayable(&charger_data.quick_poll_work, quick_poll_end_handler);
    
    /* 读取初始状态 */
    int init_pin_state = read_chrg_pin();
    if (init_pin_state >= 0) {
        update_charging_state(init_pin_state);
        LOG_INF("初始状态: %s (引脚=%d)", 
               state_strings[charger_data.current_state],
               init_pin_state);
        
        /* 记录初始状态变化时间 */
        charger_data.last_change_time = k_uptime_get();
    } else {
        LOG_ERR("读取初始引脚状态失败: %d", init_pin_state);
    }
    
    /* 启动轮询工作队列 */
    k_work_schedule(&charger_data.poll_work, K_SECONDS(charger_data.poll_interval_sec));
    
    LOG_INF("TP4056 CHRG引脚监测器初始化完成");
    LOG_INF("初始轮询间隔: %u秒", charger_data.poll_interval_sec);
    LOG_INF("快速轮询持续时间: %u秒", QUICK_POLL_DURATION_SEC);
    LOG_INF("设计特点: 中断始终启用 + 智能轮询调整");
    
    return 0;
}

/* ============== 公共API实现 ============== */

charging_state_t charging_status_get_state(void)
{
    return charger_data.current_state;
}

bool charging_status_is_charging(void)
{
    return (charger_data.current_state == CHARGING_STATE_CHARGING);
}

int charging_status_get_raw_pin_state(void)
{
    return charger_data.raw_pin_state;
}

const char* charging_status_get_state_string(void)
{
    if (charger_data.current_state <= CHARGING_STATE_NOT_CHARGING) {
        return state_strings[charger_data.current_state];
    }
    return "无效状态";
}

void charging_status_update_now(void)
{
    LOG_INF("手动更新请求");
    
    int pin_state = read_chrg_pin();
    if (pin_state >= 0) {
        /* 手动更新时，暂时进入快速轮询模式 */
        start_quick_poll_mode();
        
        update_charging_state(pin_state);
        LOG_INF("当前状态: %s (引脚=%d)", 
               state_strings[charger_data.current_state],
               pin_state);
    }
}

uint32_t charging_status_get_change_count(void)
{
    return charger_data.state_change_count;
}

void charging_status_get_history(charging_state_t *current, 
                                charging_state_t *last)
{
    if (current) *current = charger_data.current_state;
    if (last) *last = charger_data.last_state;
}

/* 设置轮询模式 */
void charging_status_set_poll_mode(bool quick_mode)
{
    if (quick_mode) {
        start_quick_poll_mode();
        LOG_INF("手动启用快速轮询模式");
    } else {
        end_quick_poll_mode();
        LOG_INF("手动禁用快速轮询模式");
    }
}

/* 获取当前轮询间隔 */
uint32_t charging_status_get_poll_interval(void)
{
    return charger_data.poll_interval_sec;
}

/* 获取中断计数 */
uint32_t charging_status_get_interrupt_count(void)
{
    return charger_data.interrupt_count;
}

/* 获取状态变化时间（毫秒） */
int64_t charging_status_get_last_change_time(void)
{
    return charger_data.last_change_time;
}

/* 检查是否在快速轮询模式 */
bool charging_status_is_quick_polling(void)
{
    return charger_data.is_quick_polling;
}

int charging_status_reinit(void)
{
    LOG_INF("重新初始化充电状态监测器");
    
    /* 停止所有工作队列 */
    k_work_cancel_delayable(&charger_data.work);
    k_work_cancel_delayable(&charger_data.poll_work);
    k_work_cancel_delayable(&charger_data.quick_poll_work);
    
    /* 重新初始化 */
    return charging_status_init();
}

/* Zephyr初始化宏 */
SYS_INIT(charging_status_init, APPLICATION, 90);