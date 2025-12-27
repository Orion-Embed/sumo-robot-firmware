#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef enum {
  BOUNDARY_NONE = 0,
  BOUNDARY_FRONT_LEFT = (1 << 0),
  BOUNDARY_FRONT_RIGHT = (1 << 1),
  BOUNDARY_REAR_LEFT = (1 << 2),
  BOUNDARY_REAR_RIGHT = (1 << 3),
  BOUNDARY_FRONT = (BOUNDARY_FRONT_LEFT | BOUNDARY_FRONT_RIGHT),
  BOUNDARY_REAR = (BOUNDARY_REAR_LEFT | BOUNDARY_REAR_RIGHT),
  BOUNDARY_LEFT = (BOUNDARY_FRONT_LEFT | BOUNDARY_REAR_LEFT),
  BOUNDARY_RIGHT = (BOUNDARY_FRONT_RIGHT | BOUNDARY_REAR_RIGHT),
  BOUNDARY_ALL = (BOUNDARY_FRONT_LEFT | BOUNDARY_FRONT_RIGHT |
                  BOUNDARY_REAR_LEFT | BOUNDARY_REAR_RIGHT)
} boundary_detection_t;

typedef struct {
  boundary_detection_t boundaries;
  bool front_left;
  bool front_right;
  bool rear_left;
  bool rear_right;
  uint32_t timestamp;
} boundary_data_t;

void ir_sensor_init(void);
void ir_sensor_read(boundary_data_t *data);
boundary_detection_t ir_sensor_check_boundary(void);
bool ir_sensor_is_near_boundary(void);
const char *ir_sensor_get_boundary_string(boundary_detection_t boundaries);
void ir_sensor_print_data(boundary_data_t *data);

#endif
