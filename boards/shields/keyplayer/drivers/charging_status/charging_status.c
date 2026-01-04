/*
 * TP4056 CHRG引脚状态监测驱动
 * 硬编码使用nRF52840 P1.09引脚
 * 专注于引脚电平读取和日志输出
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include "charging_status.h"  /* 包含头文件 */

/* 硬编码日志级别 - 不需要Kconfig */
#define CHARGING_STATUS_LOG_LEVEL 4  /* DEBUG级别，查看更多日志 */

LOG_MODULE_REGISTER(charging_status, CHARGING_STATUS_LOG_LEVEL);

/* 硬编码配置 - P1.09引脚 */
#define CHRG_GPIO_PORT DT_NODELABEL(gpio1)  /* GPIO1外设 */
#define CHRG_GPIO_PIN  9                    /* P1.09引脚 */

/* 状态字符串映射 - 与头文件中的枚举对应 */
static const char* state_strings[] = {
    "UNKNOWN",
    "CHARGING",
    "NOT_CHARGING"
};

/* 状态结构体 */
struct charging_status_data {
    const struct device *gpio_dev;     /* GPIO设备指针 */
    struct gpio_callback gpio_cb;      /* GPIO回调 */
    struct k_work_delayable work;      /* 工作队列（用于防抖） */
    struct k_work_delayable poll_work; /* 轮询工作队列 */
    charging_state_t current_state;    /* 当前充电状态 */
    charging_state_t last_state;       /* 上一次充电状态 */
    int raw_pin_state;                 /* 引脚原始电平 */
    uint32_t state_change_count;       /* 状态变化计数 */
};

/* 全局实例 */
static struct charging_status_data charger_data;

/* 内部函数声明 */
static void chrg_gpio_callback(const struct device *dev,
                               struct gpio_callback *cb,
                               uint32_t pins);
static void debounce_work_handler(struct k_work *work);
static void poll_work_handler(struct k_work *work);
static void update_charging_state(int pin_state);
static void log_current_status(void);
static int read_chrg_pin(void);

/* GPIO中断回调函数 */
static void chrg_gpio_callback(const struct device *dev,
                               struct gpio_callback *cb,
                               uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    
    if (pins & BIT(CHRG_GPIO_PIN)) {
        LOG_DBG("GPIO interrupt on P1.09");
        k_work_reschedule(&charger_data.work, K_MSEC(100));
    }
}

/* 防抖工作队列处理函数 */
static void debounce_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    
    int pin_state = read_chrg_pin();
    
    if (pin_state < 0) {
        LOG_ERR("Failed to read CHRG pin");
        return;
    }
    
    update_charging_state(pin_state);
    log_current_status();
}

/* 轮询工作队列处理函数 */
static void poll_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    
    int pin_state = read_chrg_pin();
    
    if (pin_state >= 0) {
        /* 检查状态是否变化 */
        int expected = (charger_data.current_state == CHARGING_STATE_CHARGING) ? 0 : 1;
        
        if (pin_state != expected) {
            LOG_WRN("Poll: State mismatch, expected=%d, actual=%d", expected, pin_state);
            update_charging_state(pin_state);
            log_current_status();
        } else {
            LOG_DBG("Poll: pin=%d, state=%s", 
                   pin_state, state_strings[charger_data.current_state]);
        }
    }
    
    k_work_schedule(&charger_data.poll_work, K_SECONDS(1));
}

/* 读取CHRG引脚电平 */
static int read_chrg_pin(void)
{
    if (!charger_data.gpio_dev) {
        LOG_ERR("GPIO device not initialized");
        return -ENODEV;
    }
    
    int pin_state = gpio_pin_get(charger_data.gpio_dev, CHRG_GPIO_PIN);
    
    if (pin_state < 0) {
        LOG_ERR("GPIO read error: %d", pin_state);
    }
    
    charger_data.raw_pin_state = pin_state;
    return pin_state;
}

/* 更新充电状态 */
static void update_charging_state(int pin_state)
{
    charging_state_t new_state;
    
    if (pin_state == 0) {
        new_state = CHARGING_STATE_CHARGING;
    } else {
        new_state = CHARGING_STATE_NOT_CHARGING;
    }
    
    if (new_state != charger_data.current_state) {
        charger_data.last_state = charger_data.current_state;
        charger_data.current_state = new_state;
        charger_data.state_change_count++;
        
        LOG_INF("State changed: %s -> %s",
               state_strings[charger_data.last_state],
               state_strings[charger_data.current_state]);
    }
}

/* 记录当前状态到日志 */
static void log_current_status(void)
{
    LOG_INF("CHRG Pin Status:");
    LOG_INF("  GPIO: P1.09 (GPIO1_09)");
    LOG_INF("  Raw Level: %d", charger_data.raw_pin_state);
    LOG_INF("  Level: %s", charger_data.raw_pin_state == 0 ? "LOW" : "HIGH");
    LOG_INF("  State: %s", state_strings[charger_data.current_state]);
    LOG_INF("  Changes: %u", charger_data.state_change_count);
}

/* 初始化函数 */
static int charging_status_init(void)
{
    int ret;
    
    LOG_INF("Initializing TP4056 CHRG Monitor on P1.09");
    
    /* 初始化数据结构 */
    charger_data.current_state = CHARGING_STATE_UNKNOWN;
    charger_data.last_state = CHARGING_STATE_UNKNOWN;
    charger_data.raw_pin_state = -1;
    charger_data.state_change_count = 0;
    
    /* 获取GPIO1设备 */
    charger_data.gpio_dev = DEVICE_DT_GET(CHRG_GPIO_PORT);
    if (!device_is_ready(charger_data.gpio_dev)) {
        LOG_ERR("GPIO1 not ready");
        return -ENODEV;
    }
    
    /* 配置P1.09为输入模式，启用上拉 */
    ret = gpio_pin_configure(charger_data.gpio_dev,
                            CHRG_GPIO_PIN,
                            GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) {
        LOG_ERR("Failed to configure P1.09: %d", ret);
        return ret;
    }
    
    /* 配置中断 */
    ret = gpio_pin_interrupt_configure(charger_data.gpio_dev,
                                      CHRG_GPIO_PIN,
                                      GPIO_INT_EDGE_BOTH);
    if (ret < 0) {
        LOG_ERR("Failed to configure interrupt: %d", ret);
        return ret;
    }
    
    /* 设置回调 */
    gpio_init_callback(&charger_data.gpio_cb,
                      chrg_gpio_callback,
                      BIT(CHRG_GPIO_PIN));
    
    ret = gpio_add_callback(charger_data.gpio_dev, &charger_data.gpio_cb);
    if (ret < 0) {
        LOG_ERR("Failed to add callback: %d", ret);
        return ret;
    }
    
    /* 初始化工作队列 */
    k_work_init_delayable(&charger_data.work, debounce_work_handler);
    k_work_init_delayable(&charger_data.poll_work, poll_work_handler);
    
    /* 读取初始状态 */
    int init_pin_state = read_chrg_pin();
    if (init_pin_state >= 0) {
        update_charging_state(init_pin_state);
        LOG_INF("Initial state: pin=%d, state=%s",
               init_pin_state, state_strings[charger_data.current_state]);
    }
    
    /* 启动轮询 */
    k_work_schedule(&charger_data.poll_work, K_SECONDS(1));
    
    LOG_INF("CHRG Monitor initialized successfully");
    return 0;
}

/* 公共API实现 */

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
    return "INVALID";
}

void charging_status_update_now(void)
{
    LOG_INF("Manual update requested");
    int pin_state = read_chrg_pin();
    if (pin_state >= 0) {
        update_charging_state(pin_state);
        log_current_status();
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

int charging_status_reinit(void)
{
    LOG_INF("Reinitializing...");
    k_work_cancel_delayable(&charger_data.work);
    k_work_cancel_delayable(&charger_data.poll_work);
    return charging_status_init();
}

/* 初始化 */
SYS_INIT(charging_status_init, APPLICATION, 90);