target_sources(app PRIVATE
    #src/charge_monitor.c
    #src/charging_backlight_controller.c
    #src/battery_better.c
    drivers/charger/charging_monitor.c
    drivers/layer_state/behavior_layer_led_indicator.c
)
