#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

typedef enum {
  ENEMY_NONE,
  ENEMY_FRONT,
  ENEMY_REAR,
  ENEMY_BOTH
} enemy_position_t;

typedef struct {
  enemy_position_t enemy_position;
  float front_distance; // cm
  float rear_distance;  // cm
  uint32_t timestamp;
} enemy_data_t;

// Ultrasonic sensor structure
typedef struct {
  gpio_num_t trigger_pin;
  gpio_num_t echo_pin;
  uint32_t timeout_us;
  SemaphoreHandle_t pulse_sem;
  uint64_t pulse_start;
  uint64_t pulse_end;
} ultrasonic_sensor_t;

void ultrasonic_init(void);
void ultrasonic_read_both(enemy_data_t *data);
enemy_position_t ultrasonic_detect_enemy(float attack_threshold_cm);
bool ultrasonic_is_enemy_in_range(float threshold_cm);
const char *ultrasonic_get_enemy_position_string(enemy_position_t position);
void ultrasonic_print_data(enemy_data_t *data);

// New interrupt-based functions
float ultrasonic_measure_single(ultrasonic_sensor_t *sensor);
void ultrasonic_sensor_init(ultrasonic_sensor_t *sensor, gpio_num_t trig,
                            gpio_num_t echo, uint32_t timeout);

#endif
