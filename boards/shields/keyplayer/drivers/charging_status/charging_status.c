/*
 * TP4056 CHRG引脚状态监测驱动 - 修正版
 * 硬编码使用nRF52840 P1.09引脚
 * 专注于引脚电平读取和日志输出，增加强防抖机制
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include "charging_status.h"  /* 包含头文件 */

/* 硬编码日志级别 - 设置为INFO级别，减少调试日志 */
#define CHARGING_STATUS_LOG_LEVEL 3  /* INFO级别 */

LOG_MODULE_REGISTER(charging_status, CHARGING_STATUS_LOG_LEVEL);

/* 硬编码配置 - P1.09引脚 */
#define CHRG_GPIO_PORT DT_NODELABEL(gpio1)  /* GPIO1外设 */
#define CHRG_GPIO_PIN  9                    /* P1.09引脚 */

/* 防抖配置 */
#define DEBOUNCE_DELAY_MS 500    /* 防抖延迟500ms */
#define POLL_INTERVAL_SEC 2      /* 轮询间隔2秒 */
#define MIN_STABLE_COUNT 3       /* 最小稳定次数 */

/* 状态结构体 */
struct charging_status_data {
    const struct device *gpio_dev;     /* GPIO设备指针 */
    struct gpio_callback gpio_cb;      /* GPIO回调 */
    struct k_work_delayable work;      /* 防抖工作队列 */
    struct k_work_delayable poll_work; /* 轮询工作队列 */
    
    charging_state_t current_state;    /* 当前充电状态 */
    charging_state_t last_reported_state; /* 上次报告的状态 */
    int raw_pin_state;                 /* 引脚原始电平 */
    int last_pin_state;                /* 上一次引脚电平 */
    uint32_t state_change_count;       /* 状态变化计数 */
    uint32_t interrupt_count;          /* 中断计数（用于调试） */
    
    /* 防抖相关 */
    uint8_t stable_count;              /* 稳定计数 */
    bool is_debouncing;                /* 是否正在防抖 */
};

/* 全局实例 */
static struct charging_status_data charger_data;

/* 状态字符串映射 - 与头文件中的枚举对应 */
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
static void update_charging_state(int pin_state, bool force_report);
static void report_state_change(charging_state_t old_state, charging_state_t new_state);
static int read_chrg_pin(void);
static void init_charging_status(void);

/* GPIO中断回调函数 */
static void chrg_gpio_callback(const struct device *dev,
                               struct gpio_callback *cb,
                               uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    
    if (pins & BIT(CHRG_GPIO_PIN)) {
        /* 增加中断计数 */
        charger_data.interrupt_count++;
        
        /* 仅在调试时记录中断日志 */
        LOG_DBG("GPIO中断 #%u", charger_data.interrupt_count);
        
        /* 如果已经在防抖中，取消之前的防抖工作，重新开始 */
        if (charger_data.is_debouncing) {
            k_work_cancel_delayable(&charger_data.work);
        }
        
        /* 标记为防抖中 */
        charger_data.is_debouncing = true;
        
        /* 调度防抖工作 */
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
    
    /* 更新状态（不强制报告） */
    update_charging_state(pin_state, false);
    
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
        /* 更新状态（强制报告，因为轮询间隔较长） */
        update_charging_state(pin_state, true);
        
        /* 记录轮询日志（仅调试） */
        LOG_DBG("轮询: 引脚=%d, 状态=%s, 中断次数=%u", 
               pin_state, 
               state_strings[charger_data.current_state],
               charger_data.interrupt_count);
    }
    
    /* 重新调度轮询工作 */
    k_work_schedule(&charger_data.poll_work, K_SECONDS(POLL_INTERVAL_SEC));
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

/* 更新充电状态 */
static void update_charging_state(int pin_state, bool force_report)
{
    charging_state_t new_state;
    
    /* 根据TP4056数据手册判断状态 */
    if (pin_state == 0) {
        new_state = CHARGING_STATE_CHARGING;
    } else {
        new_state = CHARGING_STATE_NOT_CHARGING;
    }
    
    /* 记录引脚状态变化（用于防抖） */
    if (pin_state == charger_data.last_pin_state) {
        charger_data.stable_count++;
    } else {
        charger_data.stable_count = 0;
        charger_data.last_pin_state = pin_state;
    }
    
    /* 更新原始引脚状态 */
    charger_data.raw_pin_state = pin_state;
    
    /* 只有当引脚状态稳定足够次数时，才考虑状态变化 */
    if (charger_data.stable_count >= MIN_STABLE_COUNT) {
        /* 检查状态是否真正变化 */
        if (new_state != charger_data.current_state) {
            charging_state_t old_state = charger_data.current_state;
            charger_data.current_state = new_state;
            charger_data.state_change_count++;
            
            /* 报告状态变化 */
            report_state_change(old_state, new_state);
        } else if (force_report) {
            /* 强制报告当前状态（例如轮询时） */
            LOG_INF("当前状态: %s (引脚=%d)", 
                   state_strings[charger_data.current_state],
                   pin_state);
        }
    } else {
        /* 状态不稳定，不更新状态，仅记录调试信息 */
        LOG_DBG("状态不稳定，稳定计数: %d/%d", 
               charger_data.stable_count, MIN_STABLE_COUNT);
    }
}

/* 报告状态变化 */
static void report_state_change(charging_state_t old_state, charging_state_t new_state)
{
    /* 获取引脚状态 */
    int pin_state = charger_data.raw_pin_state;
    
    /* 输出详细的充电状态变化日志 */
    LOG_INF("========================================");
    LOG_INF("充电状态发生变化！");
    LOG_INF("========================================");
    LOG_INF("旧状态: %s", state_strings[old_state]);
    LOG_INF("新状态: %s", state_strings[new_state]);
    LOG_INF("CHRG引脚电平: %d (%s)", 
           pin_state,
           pin_state == 0 ? "低电平" : "高电平");
    LOG_INF("TP4056状态: %s",
           new_state == CHARGING_STATE_CHARGING ? 
           "正在充电" : "未充电/充电完成");
    LOG_INF("总状态变化次数: %u", charger_data.state_change_count);
    LOG_INF("总中断次数: %u", charger_data.interrupt_count);
    LOG_INF("========================================");
    
    /* 更新上次报告的状态 */
    charger_data.last_reported_state = new_state;
}

/* 初始化函数 */
static int charging_status_init(void)
{
    int ret;
    
    LOG_INF("初始化TP4056 CHRG引脚监测器");
    LOG_INF("使用硬编码引脚: P1.09 (GPIO1_09)");
    LOG_INF("防抖延迟: %dms", DEBOUNCE_DELAY_MS);
    LOG_INF("轮询间隔: %d秒", POLL_INTERVAL_SEC);
    
    /* 初始化数据结构 */
    charger_data.current_state = CHARGING_STATE_UNKNOWN;
    charger_data.last_reported_state = CHARGING_STATE_UNKNOWN;
    charger_data.raw_pin_state = -1;
    charger_data.last_pin_state = -1;
    charger_data.state_change_count = 0;
    charger_data.interrupt_count = 0;
    charger_data.stable_count = 0;
    charger_data.is_debouncing = false;
    
    /* 获取GPIO1设备 */
    charger_data.gpio_dev = DEVICE_DT_GET(CHRG_GPIO_PORT);
    if (!device_is_ready(charger_data.gpio_dev)) {
        LOG_ERR("GPIO1设备未就绪");
        return -ENODEV;
    }
    
    LOG_INF("GPIO1设备就绪");
    
    /* 配置P1.09引脚为输入模式，启用内部上拉电阻
     * 重要：TP4056的CHRG引脚是开漏输出，必须使用上拉电阻
     */
    ret = gpio_pin_configure(charger_data.gpio_dev,
                            CHRG_GPIO_PIN,
                            GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) {
        LOG_ERR("配置P1.09失败: %d", ret);
        return ret;
    }
    
    LOG_INF("P1.09配置为输入模式，启用上拉电阻");
    
    /* 配置GPIO中断 - 双边沿触发
     * 注意：这里使用边沿触发而不是电平触发，避免持续中断
     */
    ret = gpio_pin_interrupt_configure(charger_data.gpio_dev,
                                      CHRG_GPIO_PIN,
                                      GPIO_INT_EDGE_BOTH);
    if (ret < 0) {
        LOG_ERR("配置中断失败: %d", ret);
        return ret;
    }
    
    LOG_INF("GPIO中断配置为双边沿触发");
    
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
    
    LOG_INF("GPIO回调注册成功");
    
    /* 初始化工作队列 */
    k_work_init_delayable(&charger_data.work, debounce_work_handler);
    k_work_init_delayable(&charger_data.poll_work, poll_work_handler);
    
    /* 读取初始状态 */
    int init_pin_state = read_chrg_pin();
    if (init_pin_state >= 0) {
        /* 初始状态强制报告 */
        update_charging_state(init_pin_state, true);
        LOG_INF("初始状态: 引脚=%d, 状态=%s", 
               init_pin_state,
               state_strings[charger_data.current_state]);
    } else {
        LOG_ERR("读取初始引脚状态失败: %d", init_pin_state);
    }
    
    /* 启动轮询工作队列 */
    k_work_schedule(&charger_data.poll_work, K_SECONDS(POLL_INTERVAL_SEC));
    
    LOG_INF("TP4056 CHRG引脚监测器初始化完成");
    
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
        /* 手动更新时强制报告 */
        update_charging_state(pin_state, true);
    }
}

uint32_t charging_status_get_change_count(void)
{
    return charger_data.state_change_count;
}

void charging_status_get_history(charging_state_t *current, 
                                charging_state_t *last_reported)
{
    if (current) *current = charger_data.current_state;
    if (last_reported) *last_reported = charger_data.last_reported_state;
}

int charging_status_reinit(void)
{
    LOG_INF("重新初始化充电状态监测器");
    
    /* 停止所有工作队列 */
    k_work_cancel_delayable(&charger_data.work);
    k_work_cancel_delayable(&charger_data.poll_work);
    
    /* 重置标志 */
    charger_data.is_debouncing = false;
    
    /* 重新初始化 */
    return charging_status_init();
}

/* Zephyr初始化宏 */
SYS_INIT(charging_status_init, APPLICATION, 90);