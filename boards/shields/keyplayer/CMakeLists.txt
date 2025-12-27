target_sources(app PRIVATE
    #src/charge_monitor.c
    #src/charging_backlight_controller.c
    #src/battery_better.c
    drivers/charger/charger_tp4056.c
    drivers/layer_state/layer_state_manager.c
    drivers/layer_state/layer_state_init.c
)
