#include "ir_sensor.h"
#include "pin_config.h"
#include <stdio.h>
#include <string.h>

void ir_sensor_init(void) {
  gpio_config_t io_conf = {
      .pin_bit_mask = (1ULL << IR_FRONT_LEFT) | (1ULL << IR_FRONT_RIGHT) |
                      (1ULL << IR_REAR_LEFT) | (1ULL << IR_REAR_RIGHT),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE, // Use pull-up, assume active low
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_config(&io_conf);
}

void ir_sensor_read(boundary_data_t *data) {
  data->front_left = gpio_get_level(IR_FRONT_LEFT); // Active low
  data->front_right = gpio_get_level(IR_FRONT_RIGHT);
  data->rear_left = gpio_get_level(IR_REAR_LEFT);
  data->rear_right = gpio_get_level(IR_REAR_RIGHT);
  data->timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;

  // Calculate boundary flags
  data->boundaries = BOUNDARY_NONE;
  if (data->front_left)
    data->boundaries |= BOUNDARY_FRONT_LEFT;
  if (data->front_right)
    data->boundaries |= BOUNDARY_FRONT_RIGHT;
  if (data->rear_left)
    data->boundaries |= BOUNDARY_REAR_LEFT;
  if (data->rear_right)
    data->boundaries |= BOUNDARY_REAR_RIGHT;
}

boundary_detection_t ir_sensor_check_boundary(void) {
  boundary_data_t data;
  ir_sensor_read(&data);
  return data.boundaries;
}

bool ir_sensor_is_near_boundary(void) {
  boundary_detection_t boundaries = ir_sensor_check_boundary();
  return boundaries != BOUNDARY_NONE;
}

const char *ir_sensor_get_boundary_string(boundary_detection_t boundaries) {
  static char buffer[100];
  buffer[0] = '\0';

  if (boundaries == BOUNDARY_NONE)
    return "NONE";

  if (boundaries & BOUNDARY_FRONT_LEFT)
    strcat(buffer, "FRONT_LEFT ");
  if (boundaries & BOUNDARY_FRONT_RIGHT)
    strcat(buffer, "FRONT_RIGHT ");
  if (boundaries & BOUNDARY_REAR_LEFT)
    strcat(buffer, "REAR_LEFT ");
  if (boundaries & BOUNDARY_REAR_RIGHT)
    strcat(buffer, "REAR_RIGHT ");

  return buffer;
}

void ir_sensor_print_data(boundary_data_t *data) {
  printf("BOUNDARY: FL=%d FR=%d RL=%d RR=%d | Detected: %s | Time: %lums\n",
         data->front_left, data->front_right, data->rear_left, data->rear_right,
         ir_sensor_get_boundary_string(data->boundaries), data->timestamp);
}
