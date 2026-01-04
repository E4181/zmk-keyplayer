target_sources(app PRIVATE
    #src/charge_monitor.c
    #src/charging_backlight_controller.c
    #src/battery_better.c
    #drivers/charger/charging_monitor.c
    #drivers/charger/charging_state_changed.c
    #drivers/charger/charging_led.c
    #drivers/charger/charging_controller.c
    drivers/charging_status/charging_status.c
)
