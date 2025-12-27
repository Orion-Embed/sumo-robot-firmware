# Robot controller component
idf_component_register(SRCS
    "src/robot_controller.c"
    INCLUDE_DIRS
    "include"
    REQUIRES
    motor_driver
    ir_sensor
    ultrasonic
    config
)
