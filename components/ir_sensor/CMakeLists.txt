# IR sensor component for boundary detection
idf_component_register(SRCS
    "src/ir_sensor.c"
    INCLUDE_DIRS
    "include"
    REQUIRES
    driver config
)
