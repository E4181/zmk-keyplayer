#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/init.h>

LOG_MODULE_REGISTER(charging_backlight, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/backlight.h>
#include <zmk/events/activity_state_changed.h>
#include <zmk/activity.h>
#include "charging_monitor.h"

static struct k_work_delayable init_work;

// 充电状态变化回调函数
static void on_charging_state_changed(charging_state_t new_state)
{
    switch (new_state) {
    case CHARGING_STATE_CHARGING:
        LOG_INF("Charging detected - Turning backlight ON");
        zmk_backlight_on();
        break;
        
    case CHARGING_STATE_FULL:
        LOG_INF("Battery full - Turning backlight OFF");
        zmk_backlight_off();
        break;
        
    case CHARGING_STATE_ERROR:
        LOG_WRN("Charging monitor error - Leaving backlight unchanged");
        break;
    }
}

// ZMK活动状态变化回调（优化功耗）
static int on_activity_state_changed(const zmk_event_t *eh)
{
    struct zmk_activity_state_changed *ev = as_zmk_activity_state_changed(eh);
    
    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    
    // 根据活动状态调整充电监控
    switch (ev->state) {
    case ZMK_ACTIVITY_ACTIVE:
        // 用户活跃时恢复正常监控
        charging_monitor_resume();
        break;
        
    case ZMK_ACTIVITY_IDLE:
        // 用户空闲时降低监控频率或暂停
        charging_monitor_set_polling_interval(10000); // 空闲时10秒一次
        break;
        
    case ZMK_ACTIVITY_SLEEP:
        // 休眠时暂停监控以节省电量
        charging_monitor_pause();
        break;
    }
    
    return ZMK_EV_EVENT_BUBBLE;
}

// 延迟初始化工作函数
static void delayed_init_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    
    int ret;
    
    LOG_INF("Initializing charging backlight controller with optimizations");
    
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
    
    // 注册活动状态监听器（如果可用）
    #ifdef CONFIG_ZMK_ACTIVITY_TRIGGERS
    static bool activity_listener_registered = false;
    if (!activity_listener_registered) {
        ZMK_LISTENER(charging_backlight_activity, on_activity_state_changed);
        ZMK_SUBSCRIPTION(charging_backlight_activity, zmk_activity_state_changed);
        activity_listener_registered = true;
        LOG_DBG("Registered activity state listener");
    }
    #endif
    
    LOG_INF("Charging backlight controller initialization completed");
}

// 初始化背光控制器
static int charging_backlight_controller_init(void)
{
    // 延迟3秒初始化，确保键盘功能先启动
    k_work_init_delayable(&init_work, delayed_init_work_handler);
    k_work_reschedule(&init_work, K_SECONDS(3));
    
    LOG_INF("Charging backlight controller scheduled for initialization");
    return 0;
}

// Zephyr系统初始化
SYS_INIT(charging_backlight_controller_init, APPLICATION, 99);
