#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(charge_monitor, CONFIG_LOG_DEFAULT_LEVEL);

#define CHARGE_MONITOR_NODE DT_NODELABEL(charge_monitor)

#if DT_NODE_HAS_STATUS(CHARGE_MONITOR_NODE, okay)

static const struct gpio_dt_spec chrg_pin = GPIO_DT_SPEC_GET(CHARGE_MONITOR_NODE, chrg_gpios);

static struct gpio_callback chrg_cb_data;

static void chrg_state_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    bool is_charging = !gpio_pin_get_dt(&chrg_pin);  // Low = charging (active low)
    LOG_INF("Charging state changed: %s", is_charging ? "Charging" : "Not charging");
}

static int charge_monitor_init(void)
{
    int ret;

    if (!gpio_is_ready_dt(&chrg_pin)) {
        LOG_ERR("CHRG GPIO device not ready");
        return -ENODEV;
    }

    // Configure as input with pull-up (for open-drain high state)
    ret = gpio_pin_configure_dt(&chrg_pin, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure CHRG pin: %d", ret);
        return ret;
    }

    // Enable interrupts on both edges for state transitions
    ret = gpio_pin_interrupt_configure_dt(&chrg_pin, GPIO_INT_EDGE_BOTH);
    if (ret < 0) {
        LOG_ERR("Failed to configure interrupt on CHRG pin: %d", ret);
        return ret;
    }

    // Initialize and add callback
    gpio_init_callback(&chrg_cb_data, chrg_state_handler, BIT(chrg_pin.pin));
    ret = gpio_add_callback(chrg_pin.port, &chrg_cb_data);
    if (ret < 0) {
        LOG_ERR("Failed to add callback for CHRG pin: %d", ret);
        return ret;
    }

    // Log initial state
    chrg_state_handler(chrg_pin.port, NULL, BIT(chrg_pin.pin));

    return 0;
}

SYS_INIT(charge_monitor_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#else
#warning "Charge monitor devicetree node not enabled"
#endif