#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(charging_backlight, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/backlight.h>
#include "charging_monitor.h"

static struct k_work_delayable init_work;

// 充电状态变化回调函数
static void on_charging_state_changed(enum charging_state new_state)
{
    switch (new_state) {
    case CHARGING_STATE_CHARGING:
        LOG_INF("Charging state: CHARGING - Turning backlight ON");
        zmk_backlight_on();
        break;
        
    case CHARGING_STATE_DISCHARGING:
        LOG_INF("Charging state: DISCHARGING - Turning backlight OFF");
        zmk_backlight_off();
        break;
        
    case CHARGING_STATE_FULL:
        LOG_INF("Charging state: FULL - Turning backlight OFF");
        zmk_backlight_off();
        break;
        
    case CHARGING_STATE_ERROR:
        LOG_WRN("Charging state: ERROR - Leaving backlight unchanged");
        break;
    }
}

// 延迟初始化工作函数
static void delayed_init_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    
    int ret;
    
    LOG_INF("Starting delayed initialization of charging backlight controller");
    
    // 初始化充电监控器
    ret = charging_monitor_init();
    if (ret != 0) {
        LOG_ERR("Failed to initialize charging monitor: %d", ret);
        return;
    }
    
    // 注册回调到充电监控器
    ret = charging_monitor_register_callback(on_charging_state_changed);
    if (ret != 0) {
        LOG_ERR("Failed to register backlight callback: %d", ret);
        return;
    }
    
    LOG_INF("Charging backlight controller initialization completed");
}

// 初始化背光控制器
static int charging_backlight_controller_init(void)
{
    // 延迟3秒初始化，确保键盘功能先启动
    k_work_init_delayable(&init_work, delayed_init_work_handler);
    k_work_reschedule(&init_work, K_SECONDS(3));
    
    LOG_INF("Charging backlight controller scheduled for delayed initialization");
    return 0;
}

SYS_INIT(charging_backlight_controller_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);