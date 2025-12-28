/*
 * Charging Monitor Driver for TP4056
 * Monitors CHRG pin and controls LED indicator
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/* 定义设备树节点标签 */
#define CHARGING_MONITOR_NODE DT_NODELABEL(charging_monitor)

/* 检查设备树节点是否存在 */
#if !DT_NODE_HAS_STATUS(CHARGING_MONITOR_NODE, okay)
#error "Node 'charging_monitor' not defined in device tree"
#endif

/* 从设备树获取配置 */
#define CHRG_GPIO	DT_PHANDLE(CHARGING_MONITOR_NODE, chrg_gpios)
#define CHRG_PIN	DT_GPIO_PIN(CHARGING_MONITOR_NODE, chrg_gpios)
#define CHRG_FLAGS	DT_GPIO_FLAGS(CHARGING_MONITOR_NODE, chrg_gpios)

#define LED_GPIO	DT_PHANDLE(CHARGING_MONITOR_NODE, led_gpios)
#define LED_PIN		DT_GPIO_PIN(CHARGING_MONITOR_NODE, led_gpios)
#define LED_FLAGS	DT_GPIO_FLAGS(CHARGING_MONITOR_NODE, led_gpios)

/* 工作队列和定时器 */
static struct k_work_delayable monitor_work;
static bool is_charging = false;

/* GPIO设备指针 */
static const struct device *chrg_gpio_dev;
static const struct device *led_gpio_dev;

/* 更新LED状态 */
static void update_led_state(bool charging)
{
	int ret;
	
	if (!device_is_ready(led_gpio_dev)) {
		LOG_ERR("LED GPIO device not ready");
		return;
	}
	
	/* 充电时亮灯，不充电时灭灯 */
	ret = gpio_pin_set(led_gpio_dev, LED_PIN, charging ? 1 : 0);
	if (ret < 0) {
		LOG_ERR("Failed to set LED state: %d", ret);
	}
	
	LOG_DBG("LED set to %s", charging ? "ON" : "OFF");
}

/* 检查充电状态 */
static void check_charging_status(void)
{
	int state;
	int ret;
	
	if (!device_is_ready(chrg_gpio_dev)) {
		LOG_ERR("CHRG GPIO device not ready");
		return;
	}
	
	/* 读取CHRG引脚状态 */
	state = gpio_pin_get(chrg_gpio_dev, CHRG_PIN);
	if (state < 0) {
		LOG_ERR("Failed to read CHRG pin: %d", state);
		return;
	}
	
	/* 
	 * TP4056 CHRG引脚逻辑：
	 * - 充电时：低电平 (0)
	 * - 不充电时：高电平/高阻态 (1，因为有上拉电阻)
	 */
	bool new_charging_state = (state == 0);
	
	if (is_charging != new_charging_state) {
		is_charging = new_charging_state;
		
		LOG_INF("Charging status changed: %s", 
			new_charging_state ? "CHARGING" : "NOT CHARGING");
		
		/* 更新LED状态 */
		update_led_state(new_charging_state);
	}
}

/* 工作处理函数 */
static void monitor_work_handler(struct k_work *work)
{
	check_charging_status();
	
	/* 重新调度，每500ms检查一次 */
	k_work_reschedule(&monitor_work, K_MSEC(500));
}

/* 初始化函数 */
static int charging_monitor_init(void)
{
	int ret;
	
	LOG_DBG("Initializing charging monitor");
	
	/* 获取GPIO设备 */
	chrg_gpio_dev = DEVICE_DT_GET(CHRG_GPIO);
	led_gpio_dev = DEVICE_DT_GET(LED_GPIO);
	
	if (!device_is_ready(chrg_gpio_dev)) {
		LOG_ERR("CHRG GPIO device not ready");
		return -ENODEV;
	}
	
	if (!device_is_ready(led_gpio_dev)) {
		LOG_ERR("LED GPIO device not ready");
		return -ENODEV;
	}
	
	/* 配置CHRG引脚为输入 */
	ret = gpio_pin_configure(chrg_gpio_dev, CHRG_PIN, 
				 GPIO_INPUT | CHRG_FLAGS);
	if (ret < 0) {
		LOG_ERR("Failed to configure CHRG pin: %d", ret);
		return ret;
	}
	
	LOG_DBG("CHRG pin configured on %s pin %d", 
		chrg_gpio_dev->name, CHRG_PIN);
	
	/* 配置LED引脚为输出 */
	ret = gpio_pin_configure(led_gpio_dev, LED_PIN, 
				 GPIO_OUTPUT | LED_FLAGS);
	if (ret < 0) {
		LOG_ERR("Failed to configure LED pin: %d", ret);
		return ret;
	}
	
	LOG_DBG("LED pin configured on %s pin %d", 
		led_gpio_dev->name, LED_PIN);
	
	/* 初始关闭LED */
	ret = gpio_pin_set(led_gpio_dev, LED_PIN, 0);
	if (ret < 0) {
		LOG_ERR("Failed to set initial LED state: %d", ret);
		return ret;
	}
	
	/* 初始化工作队列 */
	k_work_init_delayable(&monitor_work, monitor_work_handler);
	
	/* 立即检查一次充电状态 */
	check_charging_status();
	
	/* 启动定期检查 */
	k_work_reschedule(&monitor_work, K_MSEC(100));
	
	LOG_INF("Charging monitor initialized successfully");
	
	return 0;
}

/* 在应用层初始化 */
SYS_INIT(charging_monitor_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);