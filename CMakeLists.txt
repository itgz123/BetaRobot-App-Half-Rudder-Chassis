set(MODULE_SOURCES
    app/app.c
    app_chassis_force/app_chassis_force.c
    app_chassis_force/app_chassis_feedback.c
    app_chassis_force/app_chassis_eskf.c
    app_chassis_force/app_chassis_fsm.c
    app_chassis_force/app_chassis_force_loop.c
    app_chassis_force/app_chassis_output.c
    app_chassis_force/app_chassis_power.c
)

set(MODULE_INCLUDE_DIRS
    .
    app
    app_chassis_force
)
