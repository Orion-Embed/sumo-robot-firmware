# Register the main component
idf_component_register(SRCS
    "main.c"
    INCLUDE_DIRS
    "."
    REQUIRES
    motor_driver
    ir_sensor
    ultrasonic
    robot_controller
    wifi_remote
    nvs_flash
    driver
    esp_timer
)

