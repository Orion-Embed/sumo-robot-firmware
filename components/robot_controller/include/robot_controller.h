#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "ir_sensor.h"
#include <stdatomic.h>
// Add to robot_controller.h
typedef enum {
  EMERGENCY_NONE,
  EMERGENCY_BOUNDARY,
  EMERGENCY_MANUAL_STOP,
  EMERGENCY_OVERLOAD
} emergency_type_t;

typedef enum {
  MODE_MANUAL,
  MODE_AUTO_FIGHT,
  MODE_DEMO,
  MODE_MOTOR_TEST
} robot_mode_t;

typedef enum {
  STATE_SEARCHING,
  STATE_ATTACKING,
  STATE_ESCAPING_BOUNDARY,
  STATE_RECOVERING,
  STATE_IDLE,
  STATE_TESTING
} robot_state_t;

typedef struct {
  robot_mode_t mode;
  robot_state_t state;
  uint8_t motor_speed;
  float attack_threshold_cm;
  bool running;
  uint32_t boundary_escape_count;
  uint32_t successful_attacks;
  emergency_type_t emergency;
  atomic_bool immediate_stop_requested; // Changed to atomic
  atomic_bool emergency_boundary_detected;
} robot_config_t;

void robot_controller_init(void);
void robot_controller_start(void);
void robot_controller_stop(void);
void robot_set_mode(robot_mode_t mode);
void robot_set_speed(uint8_t speed);
void robot_set_attack_threshold(float threshold_cm);
robot_config_t robot_get_config(void);
const char *robot_get_state_string(robot_state_t state);
void robot_run_motor_test(void);

// NEW: Emergency control functions for motor_driver.c to use
bool robot_is_emergency_boundary_detected(void);
bool robot_is_immediate_stop_requested(void);
void robot_clear_emergency_flags(void);
// static void robot_handle_aggressive_escape(boundary_data_t boundaries);
#endif
