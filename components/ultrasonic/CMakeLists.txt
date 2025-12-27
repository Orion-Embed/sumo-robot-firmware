# Ultrasonic sensor component for enemy detection
idf_component_register(SRCS
    "src/ultrasonic.c"
    INCLUDE_DIRS
    "include"
    REQUIRES
    driver
    config
    esp_timer
)


