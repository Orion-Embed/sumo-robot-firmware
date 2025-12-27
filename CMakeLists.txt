# The minimum version of CMake required to build the application
cmake_minimum_required(VERSION 3.16)

# Include the build system of the ESP-IDF
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(EXTRA_COMPONENT_DIRS
    components/motor_driver
    components/ir_sensor
    components/ultrasonic
    components/robot_controller
    components/config
    components/wifi_remote
)

# Set the project name
project(robot_fight)
