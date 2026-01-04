/*
 * ZMK兼容的TP4056充电状态检测驱动
 * 硬编码使用nRF52840 P1.09引脚
 * 兼容ZMK v3.0+
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>
#include <zmk/activity.h>

LOG_MODULE_REGISTER(charging_status, CONFIG_ZMK_LOG_LEVEL);

/* 硬编码配置 - P1.09引脚 */
#define CHRG_GPIO_PORT DT_NODELABEL(gpio1)  /* GPIO1外设 */
#define CHRG_GPIO_PIN  9                    /* P1.09引脚 */

/* 充电状态结构体 */
struct charging_status_data {
    const struct device *gpio_dev;
    struct gpio_callback gpio_cb;
    struct k_work_delayable debounce_work;
    bool is_charging;
    bool last_charging_state;
    int pin_state;
};

/* 全局实例 */
static struct charging_status_data charger_data;

/* 事件定义 */
enum charging_event {
    CHARGING_EVENT_NONE,
    CHARGING_EVENT_STARTED,
    CHARGING_EVENT_STOPPED
};

/* 内部函数声明 */
static int charging_status_init(void);
static void chrg_gpio_callback(const struct device *dev,
                               struct gpio_callback *cb,
                               uint32_t pins);
static void debounce_work_handler(struct k_work *work);
static void update_charging_state(bool new_state);
static void log_pin_state(void);
static void send_charging_event(enum charging_event event);

/* 初始化函数 - ZMK兼容的初始化 */
static int charging_status_init(void)
{
    int ret;
    
    LOG_INF("Initializing TP4056 Charging Status Monitor");
    LOG_INF("Hardcoded to P1.09 (GPIO1_09)");
    
    /* 获取GPIO设备 - 硬编码使用GPIO1 */
    charger_data.gpio_dev = DEVICE_DT_GET(CHRG_GPIO_PORT);
    if (!device_is_ready(charger_data.gpio_dev)) {
        LOG_ERR("GPIO1 device not ready");
        return -ENODEV;
    }
    
    LOG_INF("GPIO1 device ready");
    
    /* 配置P1.09为输入模式，启用内部上拉电阻 */
    ret = gpio_pin_configure(charger_data.gpio_dev,
                            CHRG_GPIO_PIN,
                            GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) {
        LOG_ERR("Failed to configure P1.09: %d", ret);
        return ret;
    }
    
    LOG_INF("P1.09 configured as input with pull-up");
    
    /* 初始化GPIO回调 */
    gpio_init_callback(&charger_data.gpio_cb,
                      chrg_gpio_callback,
                      BIT(CHRG_GPIO_PIN));
    
    /* 配置GPIO中断 - 双边沿触发 */
    ret = gpio_pin_interrupt_configure(charger_data.gpio_dev,
                                      CHRG_GPIO_PIN,
                                      GPIO_INT_EDGE_BOTH);
    if (ret < 0) {
        LOG_ERR("Failed to configure interrupt: %d", ret);
        return ret;
    }
    
    LOG_INF("GPIO interrupt configured (both edges)");
    
    /* 添加GPIO回调 */
    ret = gpio_add_callback(charger_data.gpio_dev, &charger_data.gpio_cb);
    if (ret < 0) {
        LOG_ERR("Failed to add callback: %d", ret);
        return ret;
    }
    
    LOG_INF("GPIO callback registered");
    
    /* 初始化防抖工作队列 */
    k_work_init_delayable(&charger_data.debounce_work,
                         debounce_work_handler);
    
    /* 初始读取引脚状态 */
    int init_state = gpio_pin_get(charger_data.gpio_dev, CHRG_GPIO_PIN);
    if (init_state < 0) {
        LOG_ERR("Failed to read initial pin state: %d", init_state);
    } else {
        charger_data.pin_state = init_state;
        /* TP4056: 低电平(0)=充电，高电平(1)=未充电 */
        charger_data.is_charging = (init_state == 0);
        charger_data.last_charging_state = charger_data.is_charging;
        
        LOG_INF("Initial pin state: %d", init_state);
        LOG_INF("Initial charging status: %s",
               charger_data.is_charging ? "CHARGING" : "NOT CHARGING");
        
        /* 初始日志显示 */
        log_pin_state();
    }
    
    /* 启动一个周期性工作来监测状态变化 */
    k_work_schedule(&charger_data.debounce_work, K_MSEC(1000));
    
    LOG_INF("TP4056 Charging Status Monitor initialized successfully");
    LOG_INF("Monitoring CHRG pin via P1.09");
    
    return 0;
}

/* GPIO中断回调函数 */
static void chrg_gpio_callback(const struct device *dev,
                               struct gpio_callback *cb,
                               uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    
    if (pins & BIT(CHRG_GPIO_PIN)) {
        /* 防抖处理：延迟50ms后读取状态 */
        k_work_reschedule(&charger_data.debounce_work, K_MSEC(50));
        
        /* 立即读取并记录引脚状态 */
        int current_state = gpio_pin_get(charger_data.gpio_dev, CHRG_GPIO_PIN);
        if (current_state >= 0) {
            charger_data.pin_state = current_state;
            LOG_DBG("Interrupt triggered, pin state: %d", current_state);
        }
    }
}

/* 防抖工作队列处理函数 */
static void debounce_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    
    /* 读取当前引脚状态 */
    int current_state = gpio_pin_get(charger_data.gpio_dev, CHRG_GPIO_PIN);
    
    if (current_state < 0) {
        LOG_ERR("Failed to read pin state: %d", current_state);
        /* 10秒后重试 */
        k_work_schedule(&charger_data.debounce_work, K_SECONDS(10));
        return;
    }
    
    /* 更新引脚状态 */
    charger_data.pin_state = current_state;
    
    /* 根据TP4056特性判断充电状态：
     * CHRG引脚低电平(0) = 正在充电
     * CHRG引脚高电平(1) = 未充电/充电完成
     */
    bool new_charging_state = (current_state == 0);
    
    /* 记录引脚状态到日志 */
    log_pin_state();
    
    /* 检查状态是否变化 */
    if (new_charging_state != charger_data.is_charging) {
        LOG_INF("Charging state changed: %s -> %s",
               charger_data.is_charging ? "CHARGING" : "NOT CHARGING",
               new_charging_state ? "CHARGING" : "NOT CHARGING");
        
        /* 更新状态 */
        update_charging_state(new_charging_state);
        
        /* 发送事件 */
        if (new_charging_state) {
            send_charging_event(CHARGING_EVENT_STARTED);
        } else {
            send_charging_event(CHARGING_EVENT_STOPPED);
        }
    }
    
    /* 定期检查（每5秒）以防错过中断 */
    k_work_schedule(&charger_data.debounce_work, K_SECONDS(5));
}

/* 更新充电状态 */
static void update_charging_state(bool new_state)
{
    charger_data.last_charging_state = charger_data.is_charging;
    charger_data.is_charging = new_state;
}

/* 记录引脚状态到日志 */
static void log_pin_state(void)
{
    /* TP4056 CHRG引脚电平说明：
     * 0 = 低电平 = 正在充电
     * 1 = 高电平 = 未充电/充电完成
     */
    LOG_INF("CHRG pin(P1.09) state: %d - %s", 
           charger_data.pin_state,
           charger_data.pin_state == 0 ? "LOW (CHARGING)" : "HIGH (NOT CHARGING)");
    
    LOG_INF("Charging status: %s",
           charger_data.is_charging ? "CHARGING" : "NOT CHARGING");
}

/* 发送充电事件到ZMK事件系统 */
static void send_charging_event(enum charging_event event)
{
    struct zmk_activity_state_changed *ev;
    
    ev = new_zmk_activity_state_changed();
    if (ev) {
        /* 正在充电时设置为活跃状态 */
        ev->state = charger_data.is_charging ? 
                    ZMK_ACTIVITY_ACTIVE : 
                    ZMK_ACTIVITY_IDLE;
        
        /* 发布事件 */
        ZMK_EVENT_RAISE(ev);
        
        LOG_INF("Charging event sent: %s",
               event == CHARGING_EVENT_STARTED ? "CHARGING_STARTED" : "CHARGING_STOPPED");
    }
}

/* 公共API：获取当前充电状态 */
bool charging_status_is_charging(void)
{
    return charger_data.is_charging;
}

/* 公共API：获取引脚原始状态 */
int charging_status_get_pin_state(void)
{
    return charger_data.pin_state;
}

/* 公共API：手动刷新状态 */
void charging_status_refresh(void)
{
    LOG_INF("Manual refresh requested");
    k_work_reschedule(&charger_data.debounce_work, K_MSEC(10));
}

/* 公共API：获取状态字符串 */
const char* charging_status_get_string(void)
{
    return charger_data.is_charging ? "CHARGING" : "NOT_CHARGING";
}

/* ZMK兼容的初始化 */
SYS_INIT(charging_status_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

/* 导出公共API */
ZMK_SYMBOL_EXPORT(charging_status_is_charging);
ZMK_SYMBOL_EXPORT(charging_status_get_pin_state);
ZMK_SYMBOL_EXPORT(charging_status_refresh);
ZMK_SYMBOL_EXPORT(charging_status_get_string);