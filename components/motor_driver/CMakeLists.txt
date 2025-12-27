# Motor driver component for BTS7960 drivers
idf_component_register(SRCS
    "src/motor_driver.c"
    INCLUDE_DIRS
    "include"
    PRIV_REQUIRES
    robot_controller
    driver
    esp_timer
    config
)

