/*
 * Copyright (c) 2024
 * SPDX-License-Identifier: MIT
 */

#include "charging_status.h"

LOG_MODULE_REGISTER(charging_status, CONFIG_CHARGING_STATUS_LOG_LEVEL);

/* 硬编码 GPIO 配置 - P1.09 (nRF52840) */
#define CHARGING_STATUS_PORT DEVICE_DT_GET(DT_NODELABEL(gpio1))
#define CHARGING_STATUS_PIN 9

/* 防抖和滤波参数 */
#define DEBOUNCE_DELAY_MS    100  /* 防抖延时100ms */
#define SAMPLE_COUNT         5    /* 采样次数 */
#define STABLE_THRESHOLD     4    /* 稳定阈值（5次中4次相同） */
#define POLL_INTERVAL_MS     100  /* 轮询间隔（如果不使用中断） */

/* 驱动数据结构 */
static struct {
    const struct device *gpio_dev;
    struct gpio_callback gpio_cb;
    struct k_mutex mutex;
    struct k_timer debounce_timer;
    struct k_timer poll_timer;      /* 轮询定时器 */
    bool current_state;             /* 当前稳定状态 */
    bool initialized;
    uint8_t sample_buffer[SAMPLE_COUNT];  /* 采样缓冲区 */
    uint8_t sample_index;
    int interrupt_count;            /* 中断计数（调试用） */
} charging_status_data;

/* 工作队列处理函数 */
static void process_state_change(struct k_work *work)
{
    int ret;
    bool new_state;
    int high_count = 0;
    int low_count = 0;
    
    ARG_UNUSED(work);
    
    if (!charging_status_data.initialized) {
        return;
    }
    
    LOG_DBG("Processing state change...");
    
    /* 多次采样，避免噪声 */
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        ret = gpio_pin_get(charging_status_data.gpio_dev, CHARGING_STATUS_PIN);
        if (ret < 0) {
            LOG_ERR("Failed to get GPIO state: %d", ret);
            continue;
        }
        
        if (ret == 0) {
            low_count++;
        } else {
            high_count++;
        }
        
        /* 小延迟避免连续读取 */
        k_busy_wait(100);
    }
    
    /* 判断状态 */
    if (high_count >= STABLE_THRESHOLD) {
        new_state = false;  /* 高电平 - 未充电 */
    } else if (low_count >= STABLE_THRESHOLD) {
        new_state = true;   /* 低电平 - 充电中 */
    } else {
        /* 状态不稳定，保持原状态 */
        LOG_WRN("State unstable: high=%d, low=%d", high_count, low_count);
        return;
    }
    
    /* 更新状态 */
    k_mutex_lock(&charging_status_data.mutex, K_FOREVER);
    
    if (new_state != charging_status_data.current_state) {
        charging_status_data.current_state = new_state;
        
        if (new_state) {
            LOG_INF("🔌 CHARGING STARTED (CHRG pin = LOW)");
        } else {
            LOG_INF("🔋 CHARGING STOPPED (CHRG pin = HIGH)");
        }
    }
    
    k_mutex_unlock(&charging_status_data.mutex);
}

/* 工作队列结构 */
static K_WORK_DEFINE(state_change_work, process_state_change);

/* 防抖定时器回调 */
static void debounce_timer_callback(struct k_timer *timer)
{
    ARG_UNUSED(timer);
    
    /* 提交工作到系统工作队列 */
    k_work_submit(&state_change_work);
}

/* 轮询定时器回调（不使用中断时） */
static void poll_timer_callback(struct k_timer *timer)
{
    ARG_UNUSED(timer);
    
    /* 定期检查状态 */
    k_work_submit(&state_change_work);
    
    /* 重新启动定时器 */
    k_timer_start(&charging_status_data.poll_timer, 
                  K_MSEC(POLL_INTERVAL_MS), 
                  K_MSEC(POLL_INTERVAL_MS));
}

/* GPIO 中断回调函数 */
static void charging_status_gpio_callback(const struct device *dev,
                                         struct gpio_callback *cb,
                                         uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);
    
    if (!charging_status_data.initialized) {
        return;
    }
    
    charging_status_data.interrupt_count++;
    
    LOG_DBG("GPIO interrupt triggered (count: %d)", 
            charging_status_data.interrupt_count);
    
    /* 取消之前的定时器 */
    k_timer_stop(&charging_status_data.debounce_timer);
    
    /* 重新启动防抖定时器 */
    k_timer_start(&charging_status_data.debounce_timer, 
                  K_MSEC(DEBOUNCE_DELAY_MS), 
                  K_NO_WAIT);
}

/* 配置GPIO引脚 - 尝试不同配置 */
static int configure_gpio_pin(void)
{
    int ret;
    
    LOG_INF("Configuring GPIO P1.%02d...", CHARGING_STATUS_PIN);
    
    /* 方法1：标准配置（INPUT + PULL_UP） */
    ret = gpio_pin_configure(charging_status_data.gpio_dev, 
                            CHARGING_STATUS_PIN,
                            GPIO_INPUT | GPIO_PULL_UP);
    
    if (ret == 0) {
        LOG_INF("GPIO configured as INPUT with PULL_UP");
        return 0;
    }
    
    LOG_WRN("Method 1 failed: %d", ret);
    
    /* 方法2：只配置为INPUT（依赖外部上拉） */
    ret = gpio_pin_configure(charging_status_data.gpio_dev,
                            CHARGING_STATUS_PIN,
                            GPIO_INPUT);
    
    if (ret == 0) {
        LOG_INF("GPIO configured as INPUT only (external pull-up)");
        return 0;
    }
    
    LOG_ERR("All GPIO configuration methods failed");
    return ret;
}

int charging_status_init(void)
{
    int ret;
    
    LOG_INF("========================================");
    LOG_INF("Initializing charging status driver");
    LOG_INF("========================================");
    
    /* 检查是否已经初始化 */
    if (charging_status_data.initialized) {
        LOG_INF("Driver already initialized");
        return 0;
    }
    
    /* 初始化数据结构 */
    k_mutex_init(&charging_status_data.mutex);
    charging_status_data.current_state = false;
    charging_status_data.initialized = false;
    charging_status_data.sample_index = 0;
    charging_status_data.interrupt_count = 0;
    
    /* 初始化采样缓冲区 */
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        charging_status_data.sample_buffer[i] = 1; /* 默认高电平 */
    }
    
    /* 初始化防抖定时器 */
    k_timer_init(&charging_status_data.debounce_timer, 
                 debounce_timer_callback, 
                 NULL);
    
    /* 初始化轮询定时器 */
    k_timer_init(&charging_status_data.poll_timer,
                 poll_timer_callback,
                 NULL);
    
    /* 获取 GPIO 设备 */
    charging_status_data.gpio_dev = CHARGING_STATUS_PORT;
    if (!device_is_ready(charging_status_data.gpio_dev)) {
        LOG_ERR("GPIO device (GPIO1) not ready!");
        return -ENODEV;
    }
    
    LOG_INF("GPIO1 device is ready");
    
    /* 配置GPIO引脚 */
    ret = configure_gpio_pin();
    if (ret < 0) {
        LOG_ERR("Failed to configure GPIO pin");
        return ret;
    }
    
    /* 获取初始状态 */
    ret = gpio_pin_get(charging_status_data.gpio_dev, CHARGING_STATUS_PIN);
    if (ret < 0) {
        LOG_ERR("Failed to get initial GPIO state: %d", ret);
        return ret;
    }
    
    /* 初始状态判断 */
    bool initial_state = (ret == 0); /* 低电平 = 充电中 */
    
    k_mutex_lock(&charging_status_data.mutex, K_FOREVER);
    charging_status_data.current_state = initial_state;
    k_mutex_unlock(&charging_status_data.mutex);
    
    LOG_INF("Initial GPIO state: %d (%s)", 
            ret, ret == 0 ? "LOW" : "HIGH");
    LOG_INF("Initial charging state: %s", 
            initial_state ? "CHARGING" : "NOT CHARGING");
    
    /* 配置中断 */
    LOG_INF("Configuring GPIO interrupt...");
    
    /* 初始化 GPIO 回调 */
    gpio_init_callback(&charging_status_data.gpio_cb,
                      charging_status_gpio_callback,
                      BIT(CHARGING_STATUS_PIN));
    
    /* 添加回调 */
    ret = gpio_add_callback(charging_status_data.gpio_dev, &charging_status_data.gpio_cb);
    if (ret < 0) {
        LOG_WRN("Failed to add GPIO callback: %d (will use polling mode)", ret);
    } else {
        /* 启用 GPIO 中断 */
        ret = gpio_pin_interrupt_configure(charging_status_data.gpio_dev,
                                          CHARGING_STATUS_PIN,
                                          GPIO_INT_EDGE_BOTH);
        if (ret < 0) {
            LOG_WRN("Failed to configure interrupt: %d (will use polling mode)", ret);
        } else {
            LOG_INF("GPIO interrupt enabled (EDGE_BOTH)");
        }
    }
    
    /* 启动轮询定时器（无论中断是否成功） */
    k_timer_start(&charging_status_data.poll_timer, 
                  K_MSEC(POLL_INTERVAL_MS), 
                  K_MSEC(POLL_INTERVAL_MS));
    
    LOG_INF("Polling timer started (%d ms interval)", POLL_INTERVAL_MS);
    
    charging_status_data.initialized = true;
    
    LOG_INF("========================================");
    LOG_INF("Charging status driver initialized");
    LOG_INF("========================================");
    
    return 0;
}

bool charging_status_is_charging(void)
{
    bool state;
    
    if (!charging_status_data.initialized) {
        LOG_WRN("Driver not initialized");
        return false;
    }
    
    k_mutex_lock(&charging_status_data.mutex, K_FOREVER);
    state = charging_status_data.current_state;
    k_mutex_unlock(&charging_status_data.mutex);
    
    return state;
}

int charging_status_get_raw_level(void)
{
    if (!charging_status_data.initialized) {
        LOG_WRN("Driver not initialized");
        return -1;
    }
    
    return gpio_pin_get(charging_status_data.gpio_dev, CHARGING_STATUS_PIN);
}

void charging_status_log_detailed(void)
{
    if (!charging_status_data.initialized) {
        LOG_WRN("Driver not initialized");
        return;
    }
    
    int raw_level = charging_status_get_raw_level();
    bool is_charging = charging_status_is_charging();
    
    LOG_INF("=== Charging Status Details ===");
    LOG_INF("Driver initialized: %s", charging_status_data.initialized ? "YES" : "NO");
    LOG_INF("GPIO Port: GPIO1");
    LOG_INF("GPIO Pin: %d", CHARGING_STATUS_PIN);
    
    if (raw_level >= 0) {
        LOG_INF("Raw GPIO level: %d (%s)", 
                raw_level, raw_level == 0 ? "LOW" : "HIGH");
    } else {
        LOG_INF("Raw GPIO level: ERROR (%d)", raw_level);
    }
    
    LOG_INF("Interpreted state: %s", 
            is_charging ? "CHARGING (LOW)" : "NOT CHARGING (HIGH)");
    LOG_INF("Interrupt count: %d", charging_status_data.interrupt_count);
    LOG_INF("===============================");
}

/* 系统初始化函数 - 确保驱动在系统启动时初始化 */
static int charging_status_sys_init(void)
{
    LOG_INF("System init: Starting charging status driver");
    
    int ret = charging_status_init();
    if (ret < 0) {
        LOG_ERR("System init: Failed to initialize charging status driver: %d", ret);
        return ret;
    }
    
    LOG_INF("System init: Charging status driver initialized successfully");
    
    return 0;
}

/* 使用 SYS_INIT 确保驱动在系统启动时自动初始化 */
SYS_INIT(charging_status_sys_init, POST_KERNEL, 90);