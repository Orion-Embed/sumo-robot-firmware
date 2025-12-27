#include "motor_driver.h"
#include "pin_config.h"
#include "robot_controller.h"
#include <stdio.h>

static uint8_t current_speed = 200;

// BTS7960 Motor Driver Control
void bts7960_enable(bool enable) {
  gpio_set_level(BTS1_R_EN, enable);
  gpio_set_level(BTS1_L_EN, enable);
  gpio_set_level(BTS2_R_EN, enable);
  gpio_set_level(BTS2_L_EN, enable);
}

void bts7960_set_motor_speed(uint8_t driver_id, uint8_t speed, bool direction) {
  // direction: true = forward, false = backward
  if (driver_id == 0) { // BTS7960 #1 (Left motors)
    if (direction) {
      ledc_set_duty(LEDC_LOW_SPEED_MODE, BTS7960_PWM_CH_RPWM1, speed);
      ledc_set_duty(LEDC_LOW_SPEED_MODE, BTS7960_PWM_CH_LPWM1, 0);
    } else {
      ledc_set_duty(LEDC_LOW_SPEED_MODE, BTS7960_PWM_CH_RPWM1, 0);
      ledc_set_duty(LEDC_LOW_SPEED_MODE, BTS7960_PWM_CH_LPWM1, speed);
    }
    ledc_update_duty(LEDC_LOW_SPEED_MODE, BTS7960_PWM_CH_RPWM1);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, BTS7960_PWM_CH_LPWM1);
  } else { // BTS7960 #2 (Right motors)
    if (direction) {
      ledc_set_duty(LEDC_LOW_SPEED_MODE, BTS7960_PWM_CH_RPWM2, speed);
      ledc_set_duty(LEDC_LOW_SPEED_MODE, BTS7960_PWM_CH_LPWM2, 0);
    } else {
      ledc_set_duty(LEDC_LOW_SPEED_MODE, BTS7960_PWM_CH_RPWM2, 0);
      ledc_set_duty(LEDC_LOW_SPEED_MODE, BTS7960_PWM_CH_LPWM2, speed);
    }
    ledc_update_duty(LEDC_LOW_SPEED_MODE, BTS7960_PWM_CH_RPWM2);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, BTS7960_PWM_CH_LPWM2);
  }
}

void motor_driver_init(void) {
  // Configure BTS7960 enable pins
  gpio_config_t en_conf = {
      .pin_bit_mask = (1ULL << BTS1_R_EN) | (1ULL << BTS1_L_EN) |
                      (1ULL << BTS2_R_EN) | (1ULL << BTS2_L_EN),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_config(&en_conf);

  // Configure PWM timer
  ledc_timer_config_t ledc_timer = {
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .timer_num = LEDC_TIMER_0,
      .duty_resolution = BTS7960_PWM_RES,
      .freq_hz = BTS7960_PWM_FREQ,
      .clk_cfg = LEDC_AUTO_CLK,
  };
  ledc_timer_config(&ledc_timer);

  // Configure PWM channels for BTS7960 #1 (Left motors)
  ledc_channel_config_t ledc_channel_rpwm1 = {.speed_mode = LEDC_LOW_SPEED_MODE,
                                              .channel = BTS7960_PWM_CH_RPWM1,
                                              .timer_sel = LEDC_TIMER_0,
                                              .intr_type = LEDC_INTR_DISABLE,
                                              .gpio_num = BTS1_RPWM,
                                              .duty = 0,
                                              .hpoint = 0};
  ledc_channel_config(&ledc_channel_rpwm1);

  ledc_channel_config_t ledc_channel_lpwm1 = {.speed_mode = LEDC_LOW_SPEED_MODE,
                                              .channel = BTS7960_PWM_CH_LPWM1,
                                              .timer_sel = LEDC_TIMER_0,
                                              .intr_type = LEDC_INTR_DISABLE,
                                              .gpio_num = BTS1_LPWM,
                                              .duty = 0,
                                              .hpoint = 0};
  ledc_channel_config(&ledc_channel_lpwm1);

  // Configure PWM channels for BTS7960 #2 (Right motors)
  ledc_channel_config_t ledc_channel_rpwm2 = {.speed_mode = LEDC_LOW_SPEED_MODE,
                                              .channel = BTS7960_PWM_CH_RPWM2,
                                              .timer_sel = LEDC_TIMER_0,
                                              .intr_type = LEDC_INTR_DISABLE,
                                              .gpio_num = BTS2_RPWM,
                                              .duty = 0,
                                              .hpoint = 0};
  ledc_channel_config(&ledc_channel_rpwm2);

  ledc_channel_config_t ledc_channel_lpwm2 = {.speed_mode = LEDC_LOW_SPEED_MODE,
                                              .channel = BTS7960_PWM_CH_LPWM2,
                                              .timer_sel = LEDC_TIMER_0,
                                              .intr_type = LEDC_INTR_DISABLE,
                                              .gpio_num = BTS2_LPWM,
                                              .duty = 0,
                                              .hpoint = 0};
  ledc_channel_config(&ledc_channel_lpwm2);

  // Enable BTS7960 drivers
  bts7960_enable(true);

  // Stop all motors initially
  motor_stop_all();

  printf("BTS7960 Motor Driver Initialized - 4 Motors Ready\n");
}

void motor_set_speed_all(uint8_t speed) { current_speed = speed; }

void motor_forward(uint8_t speed) {
  printf("MOVING FORWARD - Speed: %d\n", speed);
  // Both drivers forward
  bts7960_set_motor_speed(0, speed, true); // Left motors forward
  bts7960_set_motor_speed(1, speed, true); // Right motors forward
}

void motor_backward(uint8_t speed) {
  printf("MOVING BACKWARD - Speed: %d\n", speed);
  // Both drivers backward
  bts7960_set_motor_speed(0, speed, false); // Left motors backward
  bts7960_set_motor_speed(1, speed, false); // Right motors backward
}

void motor_turn_left(uint8_t speed) {
  printf("TURNING LEFT - Speed: %d\n", speed);
  // Left motors backward, right motors forward
  bts7960_set_motor_speed(0, speed, false); // Left motors backward
  bts7960_set_motor_speed(1, speed, true);  // Right motors forward
}

void motor_turn_right(uint8_t speed) {
  printf("TURNING RIGHT - Speed: %d\n", speed);
  // Left motors forward, right motors backward
  bts7960_set_motor_speed(0, speed, true);  // Left motors forward
  bts7960_set_motor_speed(1, speed, false); // Right motors backward
}

void motor_spin_left(uint8_t speed) {
  printf("SPINNING LEFT - Speed: %d\n", speed);
  // Left motors backward, right motors forward (faster turn)
  bts7960_set_motor_speed(0, speed, false); // Left motors backward
  bts7960_set_motor_speed(1, speed, true);  // Right motors forward
}

void motor_spin_right(uint8_t speed) {
  printf("SPINNING RIGHT - Speed: %d\n", speed);
  // Left motors forward, right motors backward (faster turn)
  bts7960_set_motor_speed(0, speed, true);  // Left motors forward
  bts7960_set_motor_speed(1, speed, false); // Right motors backward
}

void motor_stop(void) {
  printf("STOPPING\n");
  motor_stop_all();
}

void motor_emergency_stop(void) {
  // This function stops motors IMMEDIATELY, no questions asked

  // Disable BTS7960 drivers first (safest)
  gpio_set_level(BTS1_R_EN, 0);
  gpio_set_level(BTS1_L_EN, 0);
  gpio_set_level(BTS2_R_EN, 0);
  gpio_set_level(BTS2_L_EN, 0);

  // Then set all PWM to 0
  ledc_set_duty(LEDC_LOW_SPEED_MODE, BTS7960_PWM_CH_RPWM1, 0);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, BTS7960_PWM_CH_LPWM1, 0);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, BTS7960_PWM_CH_RPWM2, 0);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, BTS7960_PWM_CH_LPWM2, 0);

  ledc_update_duty(LEDC_LOW_SPEED_MODE, BTS7960_PWM_CH_RPWM1);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, BTS7960_PWM_CH_LPWM1);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, BTS7960_PWM_CH_RPWM2);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, BTS7960_PWM_CH_LPWM2);

  printf("⚠️ EMERGENCY MOTOR STOP ACTIVATED!\n");

  // Re-enable drivers after a short delay (for safety)
  vTaskDelay(pdMS_TO_TICKS(100));
  bts7960_enable(true);
}

void motor_stop_all(void) {
  // Check emergency conditions using accessor functions
  extern bool robot_is_emergency_boundary_detected(void);
  extern bool robot_is_immediate_stop_requested(void);

  if (robot_is_emergency_boundary_detected() ||
      robot_is_immediate_stop_requested()) {
    motor_emergency_stop();
  } else {
    // Normal stop
    bts7960_set_motor_speed(0, 0, true);
    bts7960_set_motor_speed(1, 0, true);
  }
}

void motor_set_direction(motor_direction_t direction) {
  switch (direction) {
  case MOTOR_FORWARD:
    motor_forward(current_speed);
    break;
  case MOTOR_BACKWARD:
    motor_backward(current_speed);
    break;
  case MOTOR_LEFT:
    motor_turn_left(current_speed);
    break;
  case MOTOR_RIGHT:
    motor_turn_right(current_speed);
    break;
  case MOTOR_SPIN_LEFT:
    motor_spin_left(current_speed);
    break;
  case MOTOR_SPIN_RIGHT:
    motor_spin_right(current_speed);
    break;
  case MOTOR_STOP:
  default:
    motor_stop();
    break;
  }
}

void motor_set_individual(uint8_t motor_id, uint8_t speed, bool direction) {
  // For testing individual motors
  switch (motor_id) {
  case MOTOR_FRONT_LEFT:
  case MOTOR_REAR_LEFT:
    bts7960_set_motor_speed(0, speed, direction);
    break;
  case MOTOR_FRONT_RIGHT:
  case MOTOR_REAR_RIGHT:
    bts7960_set_motor_speed(1, speed, direction);
    break;
  }
}

void motor_test_sequence(void) {
  printf("=== MOTOR TEST SEQUENCE STARTED ===\n");

  printf("Testing Forward...\n");
  motor_forward(150);
  vTaskDelay(pdMS_TO_TICKS(2000));
  motor_stop();
  vTaskDelay(pdMS_TO_TICKS(500));

  printf("Testing Backward...\n");
  motor_backward(150);
  vTaskDelay(pdMS_TO_TICKS(2000));
  motor_stop();
  vTaskDelay(pdMS_TO_TICKS(500));

  printf("Testing Left Turn...\n");
  motor_turn_left(150);
  vTaskDelay(pdMS_TO_TICKS(2000));
  motor_stop();
  vTaskDelay(pdMS_TO_TICKS(500));

  printf("Testing Right Turn...\n");
  motor_turn_right(150);
  vTaskDelay(pdMS_TO_TICKS(2000));
  motor_stop();
  vTaskDelay(pdMS_TO_TICKS(500));

  printf("Testing Spin Left...\n");
  motor_spin_left(150);
  vTaskDelay(pdMS_TO_TICKS(2000));
  motor_stop();
  vTaskDelay(pdMS_TO_TICKS(500));

  printf("Testing Spin Right...\n");
  motor_spin_right(150);
  vTaskDelay(pdMS_TO_TICKS(2000));
  motor_stop();

  printf("=== MOTOR TEST SEQUENCE COMPLETED ===\n");
}
