idf_component_register(SRCS "src/wifi_remote.c"
                    INCLUDE_DIRS "include"
                    REQUIRES
                    robot_controller
                    motor_driver
                    ir_sensor
                    ultrasonic
                    config
                    esp_event
                    esp_wifi
                    esp_http_server
                    esp_netif
                    lwip
                    json
)
