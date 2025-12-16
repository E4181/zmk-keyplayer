#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(charging_monitor, CONFIG_ZMK_LOG_LEVEL);

#include "charging_monitor.h"

// 硬编码GPIO配置：使用P1.09 (GPIO1 pin 9)
#define CHARGING_GPIO_PORT      DT_NODELABEL(gpio1)  // GPIO1设备
#define CHARGING_GPIO_PIN       9                    // P1.09
#define CHARGING_GPIO_FLAGS     (GPIO_ACTIVE_LOW | GPIO_PULL_UP)  // 低电平有效，上拉

// 轮询间隔配置（毫秒）- 直接硬编码
#define POLL_INTERVAL_CHARGING_MS   2000   // 充电中：2秒
#define POLL_INTERVAL_FULL_MS      10000   // 充满：10秒
#define POLL_INTERVAL_ERROR_MS     30000   // 错误：30秒
#define POLL_INTERVAL_INTERRUPT_MS 30000   // 中断模式下的后备轮询间隔

// 空闲检测配置
#define IDLE_TIMEOUT_MS           30000    // 30秒无活动视为空闲
#define IDLE_MULTIPLIER           2        // 空闲时轮询间隔乘数

// 最大连续错误次数
#define MAX_CONSECUTIVE_ERRORS    5

// 防抖时间（毫秒）
#define DEBOUNCE_TIME_MS          1000     // 状态变化防抖时间

// 中断防抖时间（微秒）- 硬件防抖
#define INTERRUPT_DEBOUNCE_US     50000    // 50ms中断防抖

// 工作模式枚举
enum work_mode {
    MODE_POLLING = 0,     // 纯轮询模式
    MODE_INTERRUPT,       // 中断模式（主）+轮询（后备）
    MODE_ERROR            // 错误模式，回退到轮询
};

// 充电监控器私有数据结构（带中断支持）
struct charging_monitor_data {
    // 状态变量
    charging_state_t current_state;
    
    // 工作队列
    struct k_work_delayable status_check_work;
    struct k_work callback_work;
    struct k_work interrupt_work;      // 专门处理中断的工作项
    
    // GPIO相关
    const struct device *gpio_dev;
    struct gpio_callback gpio_cb;      // GPIO回调结构
    
    // 回调函数
    charging_state_changed_cb_t callback;
    
    // 统计和控制标志
    uint32_t consecutive_errors;
    uint32_t interrupt_count;          // 中断计数
    int64_t last_activity_time;
    int64_t last_state_change_time;
    int64_t last_interrupt_time;       // 上次中断时间
    bool initialized : 1;
    bool polling_active : 1;
    bool system_idle : 1;
    bool interrupt_enabled : 1;        // 中断是否启用
    bool in_interrupt : 1;             // 是否正在处理中断
    enum work_mode mode;               // 工作模式
};

// 获取私有数据实例
static struct charging_monitor_data *get_data(void)
{
    static struct charging_monitor_data data = {
        .current_state = CHARGING_STATE_ERROR,
        .callback = NULL,
        .gpio_dev = NULL,
        .consecutive_errors = 0,
        .interrupt_count = 0,
        .last_activity_time = 0,
        .last_state_change_time = 0,
        .last_interrupt_time = 0,
        .initialized = false,
        .polling_active = true,
        .system_idle = false,
        .interrupt_enabled = false,
        .in_interrupt = false,
        .mode = MODE_POLLING,
    };
    return &data;
}

// GPIO中断处理函数（在中断上下文中执行）
static void gpio_interrupt_handler(const struct device *dev, 
                                   struct gpio_callback *cb, 
                                   uint32_t pins)
{
    struct charging_monitor_data *data = CONTAINER_OF(cb, struct charging_monitor_data, gpio_cb);
    int64_t now = k_uptime_get();
    
    // 中断防抖：避免过于频繁的中断
    if ((now - data->last_interrupt_time) * 1000 < INTERRUPT_DEBOUNCE_US) {
        LOG_DBG("Interrupt debounced, too frequent");
        return;
    }
    
    data->last_interrupt_time = now;
    data->interrupt_count++;
    
    // 记录活动（中断表示可能有状态变化）
    data->last_activity_time = now;
    
    // 标记正在处理中断
    data->in_interrupt = true;
    
    // 提交中断工作项到系统工作队列（非中断上下文）
    k_work_submit(&data->interrupt_work);
    
    LOG_DBG("GPIO interrupt detected, count: %u", data->interrupt_count);
}

// 中断工作处理函数（在系统工作队列中执行，非中断上下文）
static void interrupt_work_handler(struct k_work *work)
{
    struct charging_monitor_data *data = CONTAINER_OF(work, struct charging_monitor_data, interrupt_work);
    
    if (!data->initialized || !data->gpio_dev) {
        return;
    }
    
    LOG_DBG("Processing interrupt work");
    
    // 取消可能正在排队的状态检查工作
    k_work_cancel_delayable(&data->status_check_work);
    
    // 立即执行状态检查
    k_work_reschedule(&data->status_check_work, K_NO_WAIT);
    
    // 清除中断标记
    data->in_interrupt = false;
}

// 异步回调工作函数
static void callback_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    struct charging_monitor_data *data = get_data();
    
    // 直接读取当前状态
    charging_state_t current_state = data->current_state;
    
    // 执行回调
    if (data->callback) {
        data->callback(current_state);
    }
}

// 状态变化防抖检查
static bool should_process_state_change(struct charging_monitor_data *data, 
                                       charging_state_t new_state)
{
    int64_t now = k_uptime_get();
    
    // 如果是错误状态，总是处理（需要尽快恢复）
    if (new_state == CHARGING_STATE_ERROR || 
        data->current_state == CHARGING_STATE_ERROR) {
        return true;
    }
    
    // 如果是由中断触发的状态检查，放宽防抖要求（中断表示有实际变化）
    if (data->in_interrupt) {
        // 中断模式下，防抖时间减半
        if (now - data->last_state_change_time < DEBOUNCE_TIME_MS / 2) {
            LOG_DBG("Interrupt-triggered state change debounced");
            return false;
        }
        return true;
    }
    
    // 防抖：相同状态变化至少间隔DEBOUNCE_TIME_MS
    if (now - data->last_state_change_time < DEBOUNCE_TIME_MS) {
        LOG_DBG("Polling state change debounced: %d -> %d", 
                data->current_state, new_state);
        return false;
    }
    
    return true;
}

// 智能轮询间隔计算（根据模式调整）
static uint32_t calculate_polling_interval(struct charging_monitor_data *data, 
                                          charging_state_t state, bool system_idle)
{
    uint32_t base_interval;
    
    // 根据工作模式调整基础间隔
    switch (data->mode) {
    case MODE_INTERRUPT:
        // 中断模式下，轮询作为后备，间隔较长
        base_interval = POLL_INTERVAL_INTERRUPT_MS;
        break;
    case MODE_POLLING:
    case MODE_ERROR:
    default:
        // 轮询模式下，根据状态选择间隔
        switch (state) {
        case CHARGING_STATE_CHARGING:
            base_interval = POLL_INTERVAL_CHARGING_MS;
            break;
        case CHARGING_STATE_FULL:
            base_interval = POLL_INTERVAL_FULL_MS;
            break;
        case CHARGING_STATE_ERROR:
            // 错误状态使用退避算法
            base_interval = POLL_INTERVAL_ERROR_MS * (1 + (data->consecutive_errors / 2));
            if (base_interval > 120000) base_interval = 120000; // 最大2分钟
            break;
        default:
            base_interval = POLL_INTERVAL_FULL_MS;
        }
        break;
    }
    
    // 应用空闲乘数
    if (system_idle && state != CHARGING_STATE_CHARGING) {
        base_interval *= IDLE_MULTIPLIER;
    }
    
    return base_interval;
}

// 记录活动时间
static void record_activity(void)
{
    struct charging_monitor_data *data = get_data();
    data->last_activity_time = k_uptime_get();
    
    // 如果从空闲状态恢复，记录日志
    if (data->system_idle) {
        data->system_idle = false;
        LOG_DBG("Activity detected, exiting idle mode");
    }
}

// 检查系统是否空闲
static bool check_system_idle(void)
{
    struct charging_monitor_data *data = get_data();
    int64_t now = k_uptime_get();
    bool is_idle = ((now - data->last_activity_time) > IDLE_TIMEOUT_MS);
    
    // 只有状态变化时才记录日志
    if (is_idle != data->system_idle) {
        data->system_idle = is_idle;
        LOG_DBG("System %s", is_idle ? "idle" : "active");
    }
    
    return is_idle;
}

// 尝试启用中断模式
static bool try_enable_interrupt(struct charging_monitor_data *data)
{
    int ret;
    
    // 配置GPIO中断（双边沿触发）
    ret = gpio_pin_interrupt_configure(data->gpio_dev, CHARGING_GPIO_PIN,
                                      GPIO_INT_EDGE_BOTH);
    if (ret < 0) {
        LOG_WRN("Failed to configure GPIO interrupt: %d (falling back to polling)", ret);
        return false;
    }
    
    // 初始化GPIO回调
    gpio_init_callback(&data->gpio_cb, gpio_interrupt_handler, BIT(CHARGING_GPIO_PIN));
    
    // 添加回调
    ret = gpio_add_callback(data->gpio_dev, &data->gpio_cb);
    if (ret < 0) {
        LOG_WRN("Failed to add GPIO callback: %d (falling back to polling)", ret);
        gpio_pin_interrupt_configure(data->gpio_dev, CHARGING_GPIO_PIN, GPIO_INT_DISABLE);
        return false;
    }
    
    data->interrupt_enabled = true;
    data->mode = MODE_INTERRUPT;
    LOG_INF("GPIO interrupt enabled for CHRG pin");
    
    return true;
}

// 状态检查工作函数
static void status_check_work_handler(struct k_work *work)
{
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct charging_monitor_data *data = CONTAINER_OF(dwork, struct charging_monitor_data, status_check_work);
    
    if (!data->initialized || !data->gpio_dev) {
        LOG_WRN("Charging monitor not initialized");
        k_work_reschedule(dwork, K_MSEC(POLL_INTERVAL_ERROR_MS));
        return;
    }
    
    // 检查轮询是否激活
    if (!data->polling_active) {
        LOG_DBG("Polling paused");
        return;
    }
    
    // 更新空闲状态
    bool system_idle = check_system_idle();
    
    // 读取CHRG引脚状态
    int pin_state = gpio_pin_get(data->gpio_dev, CHARGING_GPIO_PIN);
    
    if (pin_state < 0) {
        LOG_ERR("Failed to read CHRG pin: %d", pin_state);
        
        // 如果中断模式下出现错误，尝试回退到轮询模式
        if (data->mode == MODE_INTERRUPT) {
            LOG_WRN("Interrupt mode error, falling back to polling");
            data->interrupt_enabled = false;
            data->mode = MODE_POLLING;
            gpio_pin_interrupt_configure(data->gpio_dev, CHARGING_GPIO_PIN, GPIO_INT_DISABLE);
        }
        
        // 增加连续错误计数
        if (data->consecutive_errors < MAX_CONSECUTIVE_ERRORS) {
            data->consecutive_errors++;
        }
        
        // 设置为错误状态
        data->current_state = CHARGING_STATE_ERROR;
        
        // 智能调度下一次检查
        uint32_t interval = calculate_polling_interval(data, CHARGING_STATE_ERROR, system_idle);
        k_work_reschedule(dwork, K_MSEC(interval));
        return;
    }
    
    // 成功读取，清除错误计数
    data->consecutive_errors = 0;
    
    // TP4056 CHRG引脚逻辑：
    // - pin_state == 1: 引脚有效（正在充电）
    // - pin_state == 0: 引脚无效（已充满）
    charging_state_t new_state = (pin_state == 1) ? CHARGING_STATE_CHARGING : CHARGING_STATE_FULL;
    
    // 获取当前状态进行比较
    charging_state_t current_state = data->current_state;
    
    // 状态变化检测（带防抖）
    if (new_state != current_state) {
        // 检查是否需要处理这个状态变化（防抖）
        if (should_process_state_change(data, new_state)) {
            const char *old_state_str = (current_state == CHARGING_STATE_CHARGING) ? "CHARGING" : 
                                       (current_state == CHARGING_STATE_FULL) ? "FULL" : "ERROR";
            const char *new_state_str = (new_state == CHARGING_STATE_CHARGING) ? "CHARGING" : "FULL";
            const char *trigger_str = data->in_interrupt ? "interrupt" : "polling";
            
            LOG_INF("Charging state changed (%s): %s -> %s", 
                    trigger_str, old_state_str, new_state_str);
            
            // 更新状态和时间戳
            data->current_state = new_state;
            data->last_state_change_time = k_uptime_get();
            
            // 触发异步回调
            k_work_submit(&data->callback_work);
        } else {
            // 防抖过滤掉的状态变化，但仍然记录调试信息
            LOG_DBG("State change filtered by debounce: %d -> %d", 
                    current_state, new_state);
        }
    }
    
    // 智能调度下一次检查
    uint32_t interval = calculate_polling_interval(data, new_state, system_idle);
    k_work_reschedule(dwork, K_MSEC(interval));
}

// 初始化充电监控器
int charging_monitor_init(void)
{
    struct charging_monitor_data *data = get_data();
    int ret;
    
    if (data->initialized) {
        LOG_WRN("Charging monitor already initialized");
        return 0;
    }
    
    LOG_DBG("Initializing charging monitor with interrupt support");
    
    // 获取GPIO设备 - 硬编码使用GPIO1
    data->gpio_dev = DEVICE_DT_GET(CHARGING_GPIO_PORT);
    if (!data->gpio_dev) {
        LOG_ERR("Failed to get GPIO1 device");
        return -ENODEV;
    }
    
    if (!device_is_ready(data->gpio_dev)) {
        LOG_ERR("GPIO1 device not ready");
        return -ENODEV;
    }
    
    // 配置CHRG引脚为输入，上拉电阻
    ret = gpio_pin_configure(data->gpio_dev, CHARGING_GPIO_PIN, GPIO_INPUT | CHARGING_GPIO_FLAGS);
    if (ret < 0) {
        LOG_ERR("Failed to configure CHRG GPIO: %d", ret);
        return ret;
    }
    
    LOG_INF("Charging monitor configured: GPIO1 pin %d (P1.09), flags: 0x%x", 
            CHARGING_GPIO_PIN, CHARGING_GPIO_FLAGS);
    
    // 初始化工作队列
    k_work_init_delayable(&data->status_check_work, status_check_work_handler);
    k_work_init(&data->callback_work, callback_work_handler);
    k_work_init(&data->interrupt_work, interrupt_work_handler);
    
    // 尝试启用中断模式
    bool interrupt_success = try_enable_interrupt(data);
    
    if (interrupt_success) {
        LOG_INF("Charging monitor operating in interrupt mode");
    } else {
        LOG_INF("Charging monitor operating in polling mode");
        data->mode = MODE_POLLING;
    }
    
    // 设置初始活动时间
    record_activity();
    
    // 读取初始状态
    int initial_state = gpio_pin_get(data->gpio_dev, CHARGING_GPIO_PIN);
    if (initial_state >= 0) {
        data->current_state = (initial_state == 1) ? CHARGING_STATE_CHARGING : CHARGING_STATE_FULL;
        data->last_state_change_time = k_uptime_get();
        
        LOG_INF("Initial charging state: %s (mode: %s)", 
                (data->current_state == CHARGING_STATE_CHARGING) ? "CHARGING" : "FULL",
                data->interrupt_enabled ? "interrupt" : "polling");
    } else {
        LOG_ERR("Failed to read initial CHRG pin state: %d", initial_state);
        data->current_state = CHARGING_STATE_ERROR;
        data->mode = MODE_ERROR;
    }
    
    // 根据模式设置初始轮询间隔
    uint32_t initial_interval;
    if (data->mode == MODE_INTERRUPT) {
        initial_interval = POLL_INTERVAL_INTERRUPT_MS;
    } else {
        initial_interval = calculate_polling_interval(data, data->current_state, false);
    }
    
    // 开始状态监控
    k_work_reschedule(&data->status_check_work, K_MSEC(initial_interval));
    
    data->initialized = true;
    LOG_INF("Charging monitor initialized successfully");
    
    return 0;
}

// 注册回调函数
int charging_monitor_register_callback(charging_state_changed_cb_t callback)
{
    struct charging_monitor_data *data = get_data();
    
    if (!data->initialized) {
        LOG_ERR("Charging monitor not initialized");
        return -ENODEV;
    }
    
    if (!callback) {
        LOG_ERR("Callback function is NULL");
        return -EINVAL;
    }
    
    // 直接设置回调函数
    data->callback = callback;
    
    LOG_DBG("Callback registered");
    
    // 立即触发一次回调（通过工作队列）
    k_work_submit(&data->callback_work);
    
    return 0;
}

// 获取当前充电状态
charging_state_t charging_monitor_get_state(void)
{
    struct charging_monitor_data *data = get_data();
    
    if (!data->initialized) {
        return CHARGING_STATE_ERROR;
    }
    
    return data->current_state;
}

// 获取充电状态字符串
const char* charging_monitor_get_state_str(void)
{
    charging_state_t state = charging_monitor_get_state();
    
    switch (state) {
    case CHARGING_STATE_CHARGING:
        return "CHARGING";
    case CHARGING_STATE_FULL:
        return "FULL";
    case CHARGING_STATE_ERROR:
        return "ERROR";
    default:
        return "UNKNOWN";
    }
}

// 获取当前工作模式
const char* charging_monitor_get_mode_str(void)
{
    struct charging_monitor_data *data = get_data();
    
    if (!data->initialized) {
        return "UNINITIALIZED";
    }
    
    switch (data->mode) {
    case MODE_POLLING:
        return "POLLING";
    case MODE_INTERRUPT:
        return "INTERRUPT";
    case MODE_ERROR:
        return "ERROR";
    default:
        return "UNKNOWN";
    }
}

// 获取中断统计
uint32_t charging_monitor_get_interrupt_count(void)
{
    struct charging_monitor_data *data = get_data();
    
    if (!data->initialized) {
        return 0;
    }
    
    return data->interrupt_count;
}

// 手动触发状态检查
void charging_monitor_force_check(void)
{
    struct charging_monitor_data *data = get_data();
    
    if (!data->initialized || !data->polling_active) {
        return;
    }
    
    LOG_DBG("Manual state check triggered");
    record_activity();
    
    // 取消当前可能正在排队的工作，立即触发新的检查
    k_work_cancel_delayable(&data->status_check_work);
    k_work_reschedule(&data->status_check_work, K_NO_WAIT);
}
