/*
 * Copyright (c) 2024 Your Name
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(gpio_mcp23017, CONFIG_GPIO_LOG_LEVEL);

/* MCP23017 Register addresses */
#define MCP23017_IODIRA   0x00
#define MCP23017_IODIRB   0x01
#define MCP23017_IPOLA    0x02
#define MCP23017_IPOLB    0x03
#define MCP23017_GPINTENA 0x04
#define MCP23017_GPINTENB 0x05
#define MCP23017_DEFVALA  0x06
#define MCP23017_DEFVALB  0x07
#define MCP23017_INTCONA  0x08
#define MCP23017_INTCONB  0x09
#define MCP23017_IOCON    0x0A
#define MCP23017_GPPUA    0x0C
#define MCP23017_GPPUB    0x0D
#define MCP23017_INTFA    0x0E
#define MCP23017_INTFB    0x0F
#define MCP23017_INTCAPA  0x10
#define MCP23017_INTCAPB  0x11
#define MCP23017_GPIOA    0x12
#define MCP23017_GPIOB    0x13
#define MCP23017_OLATA    0x14
#define MCP23017_OLATB    0x15

struct mcp23017_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec int_gpio;
};

struct mcp23017_data {
    struct gpio_driver_data common;
    sys_slist_t callbacks;
    uint16_t output_state;
    uint16_t direction;
    uint16_t pullup;
};

static int mcp23017_read_reg(const struct device *dev, uint8_t reg, uint8_t *value)
{
    const struct mcp23017_config *config = dev->config;
    int ret;

    ret = i2c_write_read_dt(&config->i2c, &reg, 1, value, 1);
    if (ret != 0) {
        LOG_ERR("Failed to read register 0x%02x (err %d)", reg, ret);
    }

    return ret;
}

static int mcp23017_write_reg(const struct device *dev, uint8_t reg, uint8_t value)
{
    const struct mcp23017_config *config = dev->config;
    uint8_t buf[2] = { reg, value };
    int ret;

    ret = i2c_write_dt(&config->i2c, buf, sizeof(buf));
    if (ret != 0) {
        LOG_ERR("Failed to write register 0x%02x (err %d)", reg, ret);
    }

    return ret;
}

static int mcp23017_port_get_raw(const struct device *dev, uint32_t *value)
{
    uint8_t gpioa, gpiob;
    int ret;

    ret = mcp23017_read_reg(dev, MCP23017_GPIOA, &gpioa);
    if (ret != 0) {
        return ret;
    }

    ret = mcp23017_read_reg(dev, MCP23017_GPIOB, &gpiob);
    if (ret != 0) {
        return ret;
    }

    *value = (gpiob << 8) | gpioa;
    return 0;
}

static int mcp23017_port_set_masked_raw(const struct device *dev, uint32_t mask, uint32_t value)
{
    struct mcp23017_data *data = dev->data;
    uint16_t new_output;
    int ret;

    new_output = (data->output_state & ~mask) | (value & mask);
    if (new_output == data->output_state) {
        return 0;
    }

    ret = mcp23017_write_reg(dev, MCP23017_GPIOA, new_output & 0xFF);
    if (ret != 0) {
        return ret;
    }

    ret = mcp23017_write_reg(dev, MCP23017_GPIOB, (new_output >> 8) & 0xFF);
    if (ret != 0) {
        return ret;
    }

    data->output_state = new_output;
    return 0;
}

static int mcp23017_port_set_bits_raw(const struct device *dev, uint32_t mask)
{
    return mcp23017_port_set_masked_raw(dev, mask, mask);
}

static int mcp23017_port_clear_bits_raw(const struct device *dev, uint32_t mask)
{
    return mcp23017_port_set_masked_raw(dev, mask, 0);
}

static int mcp23017_port_toggle_bits(const struct device *dev, uint32_t mask)
{
    struct mcp23017_data *data = dev->data;
    return mcp23017_port_set_masked_raw(dev, mask, ~data->output_state);
}

static int mcp23017_pin_configure(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
    struct mcp23017_data *data = dev->data;
    uint8_t iodir_reg, gppu_reg;
    uint16_t pin_mask = BIT(pin);
    int ret;

    /* Configure direction */
    if ((flags & GPIO_OUTPUT) != 0) {
        data->direction &= ~pin_mask;
        
        /* Set initial output value */
        if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0) {
            data->output_state |= pin_mask;
        } else if ((flags & GPIO_OUTPUT_INIT_LOW) != 0) {
            data->output_state &= ~pin_mask;
        }
    } else {
        data->direction |= pin_mask;
    }

    /* Configure pull-up */
    if ((flags & GPIO_PULL_UP) != 0) {
        data->pullup |= pin_mask;
    } else {
        data->pullup &= ~pin_mask;
    }

    /* Write direction registers */
    iodir_reg = data->direction & 0xFF;
    ret = mcp23017_write_reg(dev, MCP23017_IODIRA, iodir_reg);
    if (ret != 0) {
        return ret;
    }

    iodir_reg = (data->direction >> 8) & 0xFF;
    ret = mcp23017_write_reg(dev, MCP23017_IODIRB, iodir_reg);
    if (ret != 0) {
        return ret;
    }

    /* Write pull-up registers */
    gppu_reg = data->pullup & 0xFF;
    ret = mcp23017_write_reg(dev, MCP23017_GPPUA, gppu_reg);
    if (ret != 0) {
        return ret;
    }

    gppu_reg = (data->pullup >> 8) & 0xFF;
    ret = mcp23017_write_reg(dev, MCP23017_GPPUB, gppu_reg);
    if (ret != 0) {
        return ret;
    }

    /* Write output state if it's an output */
    if ((flags & GPIO_OUTPUT) != 0) {
        return mcp23017_port_set_masked_raw(dev, pin_mask, data->output_state);
    }

    return 0;
}

static int mcp23017_init(const struct device *dev)
{
    const struct mcp23017_config *config = dev->config;
    struct mcp23017_data *data = dev->data;

    if (!device_is_ready(config->i2c.bus)) {
        LOG_ERR("I2C bus %s is not ready", config->i2c.bus->name);
        return -ENODEV;
    }

    /* Initialize all pins as inputs with no pull-up */
    data->direction = 0xFFFF;
    data->pullup = 0x0000;
    data->output_state = 0x0000;

    /* Configure IOCON register - sequential operation, disable slew rate */
    int ret = mcp23017_write_reg(dev, MCP23017_IOCON, 0x20);
    if (ret != 0) {
        LOG_ERR("Failed to configure IOCON register");
        return ret;
    }

    LOG_INF("MCP23017 initialized successfully");
    return 0;
}

static const struct gpio_driver_api mcp23017_driver_api = {
    .pin_configure = mcp23017_pin_configure,
    .port_get_raw = mcp23017_port_get_raw,
    .port_set_masked_raw = mcp23017_port_set_masked_raw,
    .port_set_bits_raw = mcp23017_port_set_bits_raw,
    .port_clear_bits_raw = mcp23017_port_clear_bits_raw,
    .port_toggle_bits = mcp23017_port_toggle_bits,
};

#define MCP23017_INIT(n) \
    static const struct mcp23017_config mcp23017_config_##n = { \
        .i2c = I2C_DT_SPEC_INST_GET(n), \
    }; \
    static struct mcp23017_data mcp23017_data_##n; \
    DEVICE_DT_INST_DEFINE(n, mcp23017_init, NULL, \
                  &mcp23017_data_##n, &mcp23017_config_##n, \
                  POST_KERNEL, CONFIG_GPIO_INIT_PRIORITY, \
                  &mcp23017_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MCP23017_INIT)