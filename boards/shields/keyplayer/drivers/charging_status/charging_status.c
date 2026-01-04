/*
 * TP4056 CHRG引脚状态监测驱动
 * 硬编码使用nRF52840 P1.09引脚
 * 专注于引脚电平读取和日志输出
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(charging_status, CONFIG_ZMK_LOG_LEVEL);

/* 硬编码配置 - P1.09引脚 */
#define CHRG_GPIO_PORT DT_NODELABEL(gpio1)  /* GPIO1外设 */
#define CHRG_GPIO_PIN  9                    /* P1.09引脚 */

/* 充电状态枚举 */
enum charging_state {
    CHARGING_STATE_UNKNOWN = 0,
    CHARGING_STATE_CHARGING,    /* CHRG引脚为低电平 */
    CHARGING_STATE_NOT_CHARGING /* CHRG引脚为高电平 */
};

/* 状态结构体 */
struct charging_status_data {
    const struct device *gpio_dev;     /* GPIO设备指针 */
    struct gpio_callback gpio_cb;      /* GPIO回调 */
    struct k_work_delayable work;      /* 工作队列（用于防抖） */
    struct k_work_delayable poll_work; /* 轮询工作队列 */
    enum charging_state current_state; /* 当前充电状态 */
    enum charging_state last_state;    /* 上一次充电状态 */
    int raw_pin_state;                 /* 引脚原始电平 */
    uint32_t state_change_count;       /* 状态变化计数 */
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
    
    /* 检查是否是CHRG引脚的中断 */
    if (pins & BIT(CHRG_GPIO_PIN)) {
        /* 立即读取引脚状态，用于调试 */
        int immediate_state = gpio_pin_get(charger_data.gpio_dev, CHRG_GPIO_PIN);
        LOG_DBG("GPIO interrupt triggered, immediate pin state: %d", immediate_state);
        
        /* 防抖处理：延迟100ms后更新状态 */
        k_work_reschedule(&charger_data.work, K_MSEC(100));
    }
}

/* 防抖工作队列处理函数 */
static void debounce_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    
    /* 读取当前引脚状态 */
    int pin_state = read_chrg_pin();
    
    if (pin_state < 0) {
        LOG_ERR("Failed to read CHRG pin state");
        return;
    }
    
    /* 更新状态 */
    update_charging_state(pin_state);
    
    /* 记录当前状态到日志 */
    log_current_status();
}

/* 轮询工作队列处理函数（确保不会错过中断） */
static void poll_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    
    /* 读取当前引脚状态 */
    int pin_state = read_chrg_pin();
    
    if (pin_state >= 0) {
        /* 如果状态与记录的不同，更新状态 */
        int expected_pin_state = (charger_data.current_state == CHARGING_STATE_CHARGING) ? 0 : 1;
        
        if (pin_state != expected_pin_state) {
            LOG_WRN("Poll detected state mismatch: expected=%d, actual=%d", 
                   expected_pin_state, pin_state);
            update_charging_state(pin_state);
            log_current_status();
        } else {
            /* 定期记录日志，每秒一次 */
            static uint32_t log_counter = 0;
            if ((log_counter++ % 5) == 0) { /* 每5秒记录一次 */
                LOG_DBG("Poll check: pin=%d, state=%s", 
                       pin_state, state_strings[charger_data.current_state]);
            }
        }
    }
    
    /* 每秒轮询一次 */
    k_work_schedule(&charger_data.poll_work, K_SECONDS(1));
}

/* 读取CHRG引脚电平 */
static int read_chrg_pin(void)
{
    if (!charger_data.gpio_dev) {
        return -ENODEV;
    }
    
    int pin_state = gpio_pin_get(charger_data.gpio_dev, CHRG_GPIO_PIN);
    
    if (pin_state < 0) {
        LOG_ERR("GPIO read error: %d", pin_state);
        return pin_state;
    }
    
    /* 更新原始引脚状态 */
    charger_data.raw_pin_state = pin_state;
    
    return pin_state;
}

/* 更新充电状态 */
static void update_charging_state(int pin_state)
{
    enum charging_state new_state;
    
    /* 根据TP4056数据手册：
     * CHRG引脚低电平(0) = 正在充电
     * CHRG引脚高电平(1) = 未充电/充电完成
     */
    if (pin_state == 0) {
        new_state = CHARGING_STATE_CHARGING;
    } else {
        new_state = CHARGING_STATE_NOT_CHARGING;
    }
    
    /* 检查状态是否变化 */
    if (new_state != charger_data.current_state) {
        charger_data.last_state = charger_data.current_state;
        charger_data.current_state = new_state;
        charger_data.state_change_count++;
        
        LOG_INF("Charging state changed: %s -> %s",
               state_strings[charger_data.last_state],
               state_strings[charger_data.current_state]);
        LOG_INF("Total state changes: %u", charger_data.state_change_count);
    }
}

/* 记录当前状态到日志 */
static void log_current_status(void)
{
    int pin_state = charger_data.raw_pin_state;
    const char* state_str = state_strings[charger_data.current_state];
    
    /* 详细日志输出 */
    LOG_INF("========================================");
    LOG_INF("CHRG Pin Status Report");
    LOG_INF("========================================");
    LOG_INF("GPIO Port: GPIO1, Pin: 09 (P1.09)");
    LOG_INF("Raw Pin Level: %d", pin_state);
    LOG_INF("Pin Level Description: %s", 
           pin_state == 0 ? "LOW (0V)" : "HIGH (3.3V)");
    LOG_INF("Charging State: %s", state_str);
    LOG_INF("TP4056 Interpretation: %s",
           charger_data.current_state == CHARGING_STATE_CHARGING ? 
           "Battery is CHARGING" : "Battery is NOT CHARGING");
    LOG_INF("State Change Count: %u", charger_data.state_change_count);
    LOG_INF("========================================");
}

/* 初始化函数 */
static int charging_status_init(void)
{
    int ret;
    
    LOG_INF("Initializing TP4056 CHRG Pin Monitor");
    LOG_INF("Using hardcoded pin: P1.09 (GPIO1_09)");
    
    /* 初始化数据结构 */
    charger_data.current_state = CHARGING_STATE_UNKNOWN;
    charger_data.last_state = CHARGING_STATE_UNKNOWN;
    charger_data.raw_pin_state = -1;
    charger_data.state_change_count = 0;
    
    /* 获取GPIO1设备 */
    charger_data.gpio_dev = DEVICE_DT_GET(CHRG_GPIO_PORT);
    if (!device_is_ready(charger_data.gpio_dev)) {
        LOG_ERR("GPIO1 device is not ready");
        return -ENODEV;
    }
    
    LOG_INF("GPIO1 device ready");
    
    /* 配置P1.09引脚为输入模式，启用内部上拉电阻
     * 注意：TP4056的CHRG引脚是开漏输出，需要上拉电阻
     */
    ret = gpio_pin_configure(charger_data.gpio_dev,
                            CHRG_GPIO_PIN,
                            GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) {
        LOG_ERR("Failed to configure P1.09: %d", ret);
        return ret;
    }
    
    LOG_INF("P1.09 configured as INPUT with PULL-UP");
    
    /* 配置GPIO中断 - 双边沿触发（上升沿和下降沿都触发） */
    ret = gpio_pin_interrupt_configure(charger_data.gpio_dev,
                                      CHRG_GPIO_PIN,
                                      GPIO_INT_EDGE_BOTH);
    if (ret < 0) {
        LOG_ERR("Failed to configure interrupt: %d", ret);
        return ret;
    }
    
    LOG_INF("GPIO interrupt configured (EDGE_BOTH)");
    
    /* 初始化GPIO回调函数 */
    gpio_init_callback(&charger_data.gpio_cb,
                      chrg_gpio_callback,
                      BIT(CHRG_GPIO_PIN));
    
    /* 添加回调函数 */
    ret = gpio_add_callback(charger_data.gpio_dev, &charger_data.gpio_cb);
    if (ret < 0) {
        LOG_ERR("Failed to add GPIO callback: %d", ret);
        return ret;
    }
    
    LOG_INF("GPIO callback registered");
    
    /* 初始化工作队列 */
    k_work_init_delayable(&charger_data.work, debounce_work_handler);
    k_work_init_delayable(&charger_data.poll_work, poll_work_handler);
    
    /* 读取初始状态 */
    int init_pin_state = read_chrg_pin();
    if (init_pin_state >= 0) {
        update_charging_state(init_pin_state);
        LOG_INF("Initial CHRG pin state: %d", init_pin_state);
        LOG_INF("Initial charging state: %s", 
               state_strings[charger_data.current_state]);
    }
    
    /* 启动轮询工作队列（1秒后开始，每秒执行一次） */
    k_work_schedule(&charger_data.poll_work, K_SECONDS(1));
    
    /* 立即记录一次状态 */
    log_current_status();
    
    LOG_INF("TP4056 CHRG Pin Monitor initialized successfully");
    LOG_INF("Monitoring TP4056 CHRG pin via P1.09");
    
    return 0;
}

/* 公共API：获取当前充电状态 */
enum charging_state charging_status_get_state(void)
{
    return charger_data.current_state;
}

/* 公共API：检查是否正在充电 */
bool charging_status_is_charging(void)
{
    return (charger_data.current_state == CHARGING_STATE_CHARGING);
}

/* 公共API：获取引脚原始电平 */
int charging_status_get_raw_pin_state(void)
{
    return charger_data.raw_pin_state;
}

/* 公共API：获取状态字符串 */
const char* charging_status_get_state_string(void)
{
    if (charger_data.current_state <= CHARGING_STATE_NOT_CHARGING) {
        return state_strings[charger_data.current_state];
    }
    return "INVALID";
}

/* 公共API：手动触发状态更新 */
void charging_status_update_now(void)
{
    LOG_INF("Manual update requested");
    
    /* 立即读取引脚状态 */
    int pin_state = read_chrg_pin();
    
    if (pin_state >= 0) {
        update_charging_state(pin_state);
        log_current_status();
    }
}

/* 公共API：获取状态变化次数 */
uint32_t charging_status_get_change_count(void)
{
    return charger_data.state_change_count;
}

/* 公共API：获取状态历史信息 */
void charging_status_get_history(enum charging_state *current, 
                                enum charging_state *last)
{
    if (current) *current = charger_data.current_state;
    if (last) *last = charger_data.last_state;
}

/* 公共API：强制引脚重新配置（如果需要） */
int charging_status_reinit(void)
{
    LOG_INF("Reinitializing charging status monitor");
    
    /* 停止所有工作队列 */
    k_work_cancel_delayable(&charger_data.work);
    k_work_cancel_delayable(&charger_data.poll_work);
    
    /* 重新初始化 */
    return charging_status_init();
}

/* Zephyr初始化宏 - 在应用层初始化 */
SYS_INIT(charging_status_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

/* 导出公共API函数 */
static const struct {
    enum charging_state (*get_state)(void);
    bool (*is_charging)(void);
    int (*get_raw_pin_state)(void);
    const char* (*get_state_string)(void);
    void (*update_now)(void);
    uint32_t (*get_change_count)(void);
    void (*get_history)(enum charging_state*, enum charging_state*);
    int (*reinit)(void);
} charging_status_api = {
    .get_state = charging_status_get_state,
    .is_charging = charging_status_is_charging,
    .get_raw_pin_state = charging_status_get_raw_pin_state,
    .get_state_string = charging_status_get_state_string,
    .update_now = charging_status_update_now,
    .get_change_count = charging_status_get_change_count,
    .get_history = charging_status_get_history,
    .reinit = charging_status_reinit,
};