/*
 * TP4056充电呼吸灯控制驱动
 * 硬编码使用nRF52840 P1.06引脚
 * 当检测到充电时，LED呈现呼吸灯效果
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include "charging_status.h"  /* 充电状态接口 */

LOG_MODULE_REGISTER(charging_breathe, CONFIG_ZMK_LOG_LEVEL);

/* 硬编码配置 - P1.06引脚 */
#define LED_GPIO_PORT DT_NODELABEL(gpio1)  /* GPIO1外设 */
#define LED_GPIO_PIN  6                    /* P1.06引脚 */

/* 呼吸灯配置 */
#define BREATHE_PERIOD_MS 4000    /* 呼吸周期：4秒 */
#define BREATHE_STEPS     100     /* 呼吸步数 */
#define BREATHE_INTERVAL_MS (BREATHE_PERIOD_MS / BREATHE_STEPS) /* 每步间隔 */

/* LED亮度配置 */
#define LED_OFF_DELAY_MS   50     /* LED关闭时的最小延迟 */
#define MAX_PWM_CYCLES     100    /* PWM最大周期数 */

/* 呼吸灯状态 */
typedef enum {
    BREATHE_STATE_OFF = 0,     /* 呼吸灯关闭 */
    BREATHE_STATE_BREATHING,   /* 呼吸效果中 */
    BREATHE_STATE_ON           /* 常亮状态 */
} breathe_state_t;

/* 呼吸灯结构体 */
struct charging_breathe_data {
    const struct device *gpio_dev;       /* GPIO设备指针 */
    struct k_work_delayable breathe_work; /* 呼吸灯工作队列 */
    struct k_work_delayable pwm_work;    /* PWM工作队列 */
    
    breathe_state_t current_state;       /* 当前呼吸灯状态 */
    bool led_on;                         /* LED当前是否点亮 */
    uint32_t breathe_step;               /* 呼吸步数计数器 */
    bool breathe_direction;              /* 呼吸方向：true=渐亮，false=渐暗 */
    uint32_t pwm_on_cycles;              /* PWM点亮周期数 */
    uint32_t pwm_off_cycles;             /* PWM熄灭周期数 */
    uint32_t pwm_counter;                /* PWM计数器 */
    bool pwm_state;                      /* PWM当前状态 */
    
    /* 充电状态跟踪 */
    bool last_charging_state;            /* 上次充电状态 */
    bool charging_state_changed;         /* 充电状态是否变化 */
};

/* 全局实例 */
static struct charging_breathe_data breathe_data;

/* 内部函数声明 */
static int charging_breathe_init(void);
static void breathe_work_handler(struct k_work *work);
static void pwm_work_handler(struct k_work *work);
static void update_breathing_pattern(void);
static void set_led_state(bool state);
static void start_breathing(void);
static void stop_breathing(void);
static void update_charging_state_handler(void);
static void check_charging_state(void);

/* 初始化GPIO引脚 */
static int init_led_gpio(void)
{
    int ret;
    
    /* 获取GPIO1设备 */
    breathe_data.gpio_dev = DEVICE_DT_GET(LED_GPIO_PORT);
    if (!device_is_ready(breathe_data.gpio_dev)) {
        LOG_ERR("GPIO1设备未就绪");
        return -ENODEV;
    }
    
    LOG_INF("GPIO1设备就绪，配置P1.06引脚");
    
    /* 配置P1.06引脚为输出模式，初始为低电平
     * 注意：根据LED接线方式，可能需要调整GPIO_ACTIVE_HIGH/LOW
     * 这里假设高电平点亮LED
     */
    ret = gpio_pin_configure(breathe_data.gpio_dev,
                            LED_GPIO_PIN,
                            GPIO_OUTPUT | GPIO_ACTIVE_HIGH);
    if (ret < 0) {
        LOG_ERR("配置P1.06失败: %d", ret);
        return ret;
    }
    
    /* 初始状态：LED关闭 */
    ret = gpio_pin_set(breathe_data.gpio_dev, LED_GPIO_PIN, 0);
    if (ret < 0) {
        LOG_ERR("设置LED初始状态失败: %d", ret);
        return ret;
    }
    
    LOG_INF("P1.06配置为输出模式，初始为低电平");
    breathe_data.led_on = false;
    
    return 0;
}

/* 设置LED状态 */
static void set_led_state(bool state)
{
    if (!breathe_data.gpio_dev) {
        return;
    }
    
    int ret = gpio_pin_set(breathe_data.gpio_dev, LED_GPIO_PIN, state ? 1 : 0);
    if (ret < 0) {
        LOG_ERR("设置LED状态失败: %d", ret);
    } else {
        breathe_data.led_on = state;
    }
}

/* 更新呼吸灯模式 */
static void update_breathing_pattern(void)
{
    /* 计算呼吸曲线（使用正弦波的一半，0-π） */
    float angle;
    float brightness;
    
    if (breathe_data.breathe_direction) {
        /* 渐亮阶段 */
        angle = (float)breathe_data.breathe_step / BREATHE_STEPS * M_PI;
    } else {
        /* 渐暗阶段 */
        angle = M_PI - (float)breathe_data.breathe_step / BREATHE_STEPS * M_PI;
    }
    
    /* 使用正弦函数计算亮度（0-1范围） */
    brightness = sinf(angle);
    
    /* 将亮度转换为PWM周期数（0-100） */
    breathe_data.pwm_on_cycles = (uint32_t)(brightness * MAX_PWM_CYCLES);
    breathe_data.pwm_off_cycles = MAX_PWM_CYCLES - breathe_data.pwm_on_cycles;
    
    /* 确保至少有一个周期 */
    if (breathe_data.pwm_on_cycles == 0) {
        breathe_data.pwm_on_cycles = 1;
        breathe_data.pwm_off_cycles = MAX_PWM_CYCLES - 1;
    } else if (breathe_data.pwm_off_cycles == 0) {
        breathe_data.pwm_off_cycles = 1;
        breathe_data.pwm_on_cycles = MAX_PWM_CYCLES - 1;
    }
    
    breathe_data.pwm_counter = 0;
    breathe_data.pwm_state = false;
    
    /* 调试日志 */
    LOG_DBG("呼吸步数: %u/%u, 方向: %s, 亮度: %.2f, PWM: %u/%u",
           breathe_data.breathe_step, BREATHE_STEPS,
           breathe_data.breathe_direction ? "渐亮" : "渐暗",
           brightness, breathe_data.pwm_on_cycles, breathe_data.pwm_off_cycles);
}

/* PWM工作队列处理函数 */
static void pwm_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    
    if (breathe_data.current_state != BREATHE_STATE_BREATHING) {
        /* 不在呼吸状态，停止PWM */
        set_led_state(false);
        return;
    }
    
    if (!breathe_data.pwm_state) {
        /* 当前是熄灭状态，检查是否需要点亮 */
        breathe_data.pwm_counter++;
        if (breathe_data.pwm_counter >= breathe_data.pwm_off_cycles) {
            /* 熄灭时间结束，点亮LED */
            set_led_state(true);
            breathe_data.pwm_state = true;
            breathe_data.pwm_counter = 0;
            
            /* 调度下一次PWM切换 */
            k_work_reschedule(&breathe_data.pwm_work, 
                             K_USEC(breathe_data.pwm_on_cycles * 10)); /* 10us * 周期数 */
        } else {
            /* 继续保持熄灭 */
            k_work_reschedule(&breathe_data.pwm_work, K_USEC(10)); /* 10us检查一次 */
        }
    } else {
        /* 当前是点亮状态，检查是否需要熄灭 */
        breathe_data.pwm_counter++;
        if (breathe_data.pwm_counter >= breathe_data.pwm_on_cycles) {
            /* 点亮时间结束，熄灭LED */
            set_led_state(false);
            breathe_data.pwm_state = false;
            breathe_data.pwm_counter = 0;
            
            /* 调度下一次PWM切换 */
            k_work_reschedule(&breathe_data.pwm_work, 
                             K_USEC(breathe_data.pwm_off_cycles * 10)); /* 10us * 周期数 */
        } else {
            /* 继续保持点亮 */
            k_work_reschedule(&breathe_data.pwm_work, K_USEC(10)); /* 10us检查一次 */
        }
    }
}

/* 呼吸灯工作队列处理函数 */
static void breathe_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    
    if (breathe_data.current_state != BREATHE_STATE_BREATHING) {
        /* 不在呼吸状态，停止呼吸效果 */
        return;
    }
    
    /* 更新呼吸步数 */
    if (breathe_data.breathe_direction) {
        /* 渐亮阶段 */
        breathe_data.breathe_step++;
        if (breathe_data.breathe_step >= BREATHE_STEPS) {
            breathe_data.breathe_direction = false;
            breathe_data.breathe_step = BREATHE_STEPS;
        }
    } else {
        /* 渐暗阶段 */
        if (breathe_data.breathe_step == 0) {
            breathe_data.breathe_direction = true;
            breathe_data.breathe_step = 0;
        } else {
            breathe_data.breathe_step--;
        }
    }
    
    /* 更新呼吸模式 */
    update_breathing_pattern();
    
    /* 调度下一次呼吸步进 */
    k_work_reschedule(&breathe_data.breathe_work, 
                     K_MSEC(BREATHE_INTERVAL_MS));
    
    /* 启动PWM工作（如果还没启动） */
    if (!k_work_delayable_is_pending(&breathe_data.pwm_work)) {
        breathe_data.pwm_counter = 0;
        breathe_data.pwm_state = false;
        set_led_state(false);
        k_work_reschedule(&breathe_data.pwm_work, K_USEC(10));
    }
}

/* 开始呼吸效果 */
static void start_breathing(void)
{
    if (breathe_data.current_state == BREATHE_STATE_BREATHING) {
        /* 已经在呼吸状态 */
        return;
    }
    
    LOG_INF("开始呼吸灯效果");
    
    breathe_data.current_state = BREATHE_STATE_BREATHING;
    breathe_data.breathe_step = 0;
    breathe_data.breathe_direction = true;  /* 从暗到亮开始 */
    
    /* 初始化呼吸模式 */
    update_breathing_pattern();
    
    /* 启动呼吸工作队列 */
    k_work_reschedule(&breathe_data.breathe_work, 
                     K_MSEC(BREATHE_INTERVAL_MS));
    
    /* 启动PWM工作队列 */
    breathe_data.pwm_counter = 0;
    breathe_data.pwm_state = false;
    set_led_state(false);
    k_work_reschedule(&breathe_data.pwm_work, K_USEC(10));
}

/* 停止呼吸效果 */
static void stop_breathing(void)
{
    LOG_INF("停止呼吸灯效果");
    
    /* 取消工作队列 */
    k_work_cancel_delayable(&breathe_data.breathe_work);
    k_work_cancel_delayable(&breathe_data.pwm_work);
    
    /* 关闭LED */
    set_led_state(false);
    
    breathe_data.current_state = BREATHE_STATE_OFF;
}

/* 设置为常亮状态 */
static void set_led_on(void)
{
    if (breathe_data.current_state == BREATHE_STATE_ON) {
        return;
    }
    
    LOG_INF("设置LED常亮");
    
    /* 取消工作队列 */
    k_work_cancel_delayable(&breathe_data.breathe_work);
    k_work_cancel_delayable(&breathe_data.pwm_work);
    
    /* 点亮LED */
    set_led_state(true);
    
    breathe_data.current_state = BREATHE_STATE_ON;
}

/* 检查充电状态并更新LED */
static void check_charging_state(void)
{
    static uint32_t last_check_time = 0;
    uint32_t current_time = k_uptime_get_32();
    
    /* 限制检查频率，每500ms检查一次 */
    if ((current_time - last_check_time) < 500) {
        return;
    }
    last_check_time = current_time;
    
    /* 获取当前充电状态 */
    bool is_charging = charging_status_is_charging();
    
    /* 检查状态是否变化 */
    if (is_charging != breathe_data.last_charging_state) {
        breathe_data.charging_state_changed = true;
        breathe_data.last_charging_state = is_charging;
        
        LOG_INF("充电状态变化: %s -> %s",
               breathe_data.last_charging_state ? "充电" : "未充电",
               is_charging ? "充电" : "未充电");
    }
    
    /* 根据充电状态控制LED */
    if (is_charging) {
        /* 正在充电：开始呼吸效果 */
        if (breathe_data.charging_state_changed || 
            breathe_data.current_state != BREATHE_STATE_BREATHING) {
            start_breathing();
        }
    } else {
        /* 未充电：停止呼吸效果，关闭LED */
        if (breathe_data.charging_state_changed || 
            breathe_data.current_state != BREATHE_STATE_OFF) {
            stop_breathing();
        }
    }
    
    breathe_data.charging_state_changed = false;
}

/* 充电状态更新处理函数 */
static void update_charging_state_handler(void)
{
    /* 直接检查充电状态并更新LED */
    check_charging_state();
}

/* 初始化函数 */
static int charging_breathe_init(void)
{
    int ret;
    
    LOG_INF("初始化充电呼吸灯控制");
    LOG_INF("使用硬编码引脚: P1.06 (GPIO1_06)");
    LOG_INF("呼吸周期: %dms, 步数: %d", BREATHE_PERIOD_MS, BREATHE_STEPS);
    
    /* 初始化数据结构 */
    breathe_data.current_state = BREATHE_STATE_OFF;
    breathe_data.led_on = false;
    breathe_data.breathe_step = 0;
    breathe_data.breathe_direction = true;
    breathe_data.pwm_on_cycles = 0;
    breathe_data.pwm_off_cycles = 0;
    breathe_data.pwm_counter = 0;
    breathe_data.pwm_state = false;
    breathe_data.last_charging_state = false;
    breathe_data.charging_state_changed = false;
    
    /* 初始化GPIO引脚 */
    ret = init_led_gpio();
    if (ret < 0) {
        LOG_ERR("LED GPIO初始化失败");
        return ret;
    }
    
    /* 初始化工作队列 */
    k_work_init_delayable(&breathe_data.breathe_work, breathe_work_handler);
    k_work_init_delayable(&breathe_data.pwm_work, pwm_work_handler);
    
    /* 获取初始充电状态 */
    bool init_charging = charging_status_is_charging();
    breathe_data.last_charging_state = init_charging;
    
    LOG_INF("初始充电状态: %s", init_charging ? "充电" : "未充电");
    
    /* 初始状态：LED关闭 */
    stop_breathing();
    
    LOG_INF("充电呼吸灯控制初始化完成");
    
    return 0;
}

/* ============== 公共API实现 ============== */

/* 手动控制呼吸灯 */
void charging_breathe_set_state(bool enable_breathing)
{
    if (enable_breathing) {
        start_breathing();
    } else {
        stop_breathing();
    }
}

/* 手动设置LED状态 */
void charging_breathe_set_led(bool led_on)
{
    if (led_on) {
        set_led_on();
    } else {
        stop_breathing();  /* 停止呼吸效果并关闭LED */
    }
}

/* 获取当前呼吸灯状态 */
breathe_state_t charging_breathe_get_state(void)
{
    return breathe_data.current_state;
}

/* 获取LED当前是否点亮 */
bool charging_breathe_is_led_on(void)
{
    return breathe_data.led_on;
}

/* 手动触发充电状态检查 */
void charging_breathe_check_now(void)
{
    check_charging_state();
}

/* 设置呼吸周期 */
void charging_breathe_set_period(uint32_t period_ms)
{
    /* 注意：这需要重新计算BREATHE_INTERVAL_MS，但为了简化，这里只记录 */
    LOG_INF("设置呼吸周期为: %ums", period_ms);
    /* 实际应用中需要重启呼吸效果来应用新周期 */
}

/* 重新初始化呼吸灯控制 */
int charging_breathe_reinit(void)
{
    LOG_INF("重新初始化呼吸灯控制");
    
    /* 停止所有工作队列 */
    k_work_cancel_delayable(&breathe_data.breathe_work);
    k_work_cancel_delayable(&breathe_data.pwm_work);
    
    /* 重新初始化 */
    return charging_breathe_init();
}

/* Zephyr初始化宏 */
SYS_INIT(charging_breathe_init, APPLICATION, 80);  /* 在充电状态监测之后初始化 */