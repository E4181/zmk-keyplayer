target_sources(app PRIVATE
    #src/charge_monitor.c
    #src/charging_backlight_controller.c
    #src/battery_better.c
    drivers/charger/tp4056_charger.c
    #drivers/layer_state/behavior_layer_led_indicator.c
)
