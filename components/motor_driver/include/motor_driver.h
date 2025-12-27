#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "robot_controller.h"

typedef enum {
  MOTOR_STOP,
  MOTOR_FORWARD,
  MOTOR_BACKWARD,
  MOTOR_LEFT,
  MOTOR_RIGHT,
  MOTOR_SPIN_LEFT,
  MOTOR_SPIN_RIGHT
} motor_direction_t;

typedef struct {
  motor_direction_t direction;
  uint8_t speed; // 0-255
  uint32_t duration_ms;
} motor_command_t;

// Motor control functions
void motor_driver_init(void);
void motor_set_speed_all(uint8_t speed);
void motor_set_direction(motor_direction_t direction);
void motor_forward(uint8_t speed);
void motor_backward(uint8_t speed);
void motor_turn_left(uint8_t speed);
void motor_turn_right(uint8_t speed);
void motor_spin_left(uint8_t speed);
void motor_spin_right(uint8_t speed);
void motor_stop(void);
void motor_stop_all(void);
void motor_emergency_stop(void);
// Individual motor control (for testing)
void motor_set_individual(uint8_t motor_id, uint8_t speed, bool direction);
void motor_test_sequence(void);

// BTS7960 specific functions
void bts7960_enable(bool enable);
void bts7960_set_motor_speed(uint8_t driver_id, uint8_t speed, bool direction);

#endif
