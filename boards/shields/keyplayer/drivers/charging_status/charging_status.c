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
#include <zmk/events/battery_state_changed.h>
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
    struct k_work_delayable periodic_work;
    bool is_charging;
    bool last_charging_state;
    int pin_state;
};

/* 全局实例 */
static struct charging_status_data charger_data;

/* 内部函数声明 */
static int charging_status_init(void);
static void chrg_gpio_callback(const struct device *dev,
                               struct gpio_callback *cb,
                               uint32_t pins);
static void debounce_work_handler(struct k_work *work);
static void periodic_work_handler(struct k_work *work);
static void update_charging_state(bool new_state);
static void log_pin_state(void);

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
    
    /* 初始化定期工作队列（每2秒检查一次） */
    k_work_init_delayable(&charger_data.periodic_work,
                         periodic_work_handler);
    
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
    
    /* 启动周期性工作（2秒后开始，每5秒执行一次） */
    k_work_schedule(&charger_data.periodic_work, K_MSEC(2000));
    
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
        /* 防抖处理：延迟100ms后读取状态 */
        k_work_reschedule(&charger_data.debounce_work, K_MSEC(100));
        
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
    }
}

/* 定期工作队列处理函数（确保不会错过状态变化） */
static void periodic_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    
    /* 读取当前引脚状态 */
    int current_state = gpio_pin_get(charger_data.gpio_dev, CHRG_GPIO_PIN);
    
    if (current_state < 0) {
        LOG_WRN("Failed to read pin state in periodic check: %d", current_state);
    } else if (current_state != charger_data.pin_state) {
        /* 状态变化了，触发防抖处理 */
        k_work_reschedule(&charger_data.debounce_work, K_MSEC(50));
    } else {
        /* 状态未变，记录一次日志 */
        LOG_DBG("Periodic check: pin state=%d, charging=%s",
               current_state,
               charger_data.is_charging ? "YES" : "NO");
    }
    
    /* 每5秒检查一次 */
    k_work_schedule(&charger_data.periodic_work, K_SECONDS(5));
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
    int pin_state = charger_data.pin_state;
    bool is_charging = charger_data.is_charging;
    
    LOG_INF("CHRG pin(P1.09) state: %d - %s", 
           pin_state,
           pin_state == 0 ? "LOW" : "HIGH");
    
    LOG_INF("Charging status: %s",
           is_charging ? "CHARGING" : "NOT CHARGING");
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

/* 公共API：获取状态变化历史 */
bool charging_status_has_changed(void)
{
    return charger_data.is_charging != charger_data.last_charging_state;
}

/* ZMK兼容的初始化 - 使用POST_KERNEL优先级 */
static int charging_status_init_post(const struct device *dev)
{
    ARG_UNUSED(dev);
    return charging_status_init();
}

SYS_INIT(charging_status_init_post, POST_KERNEL, 90);