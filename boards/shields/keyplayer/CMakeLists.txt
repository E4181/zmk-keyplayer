target_sources(app PRIVATE
    src/charging_monitor.c
    src/charging_backlight_controller.c
)

# 添加自定义驱动目录
zephyr_library()
zephyr_library_sources(drivers/gpio/gpio_mcp23017.c)

# 添加设备树绑定目录
list(APPEND DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR})