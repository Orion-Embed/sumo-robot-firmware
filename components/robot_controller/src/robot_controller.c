#include "robot_controller.h"
#include "ir_sensor.h"
#include "motor_driver.h"
#include "ultrasonic.h"
#include <stdio.h>
#include <string.h>

static enemy_position_t last_enemy_position = ENEMY_NONE;
static uint32_t last_seen_time = 0;
static uint32_t search_iteration = 0;
static int8_t search_direction = 1; // 1 for clockwise, -1 for counter-clockwise

static robot_config_t robot_config = {.mode = MODE_AUTO_FIGHT,
                                      .state = STATE_IDLE,
                                      .motor_speed = 200,
                                      .attack_threshold_cm = 30.0,
                                      .running = false,
                                      .boundary_escape_count = 0,
                                      .successful_attacks = 0};

static TaskHandle_t robot_task_handle = NULL;

// Add these global variables at the top of robot_controller.c:
static bool emergency_boundary_detected = false;
static SemaphoreHandle_t motor_stop_mutex = NULL;
static TaskHandle_t boundary_watchdog_handle = NULL;

// Boundary watchdog task - runs at highest priority
static void boundary_watchdog_task(void *pvParameters) {
  boundary_data_t boundary_data;

  while (1) {
    // Read IR sensors every 50ms
    ir_sensor_read(&boundary_data);

    if (boundary_data.boundaries != BOUNDARY_NONE) {
      // Set emergency flag but DON'T reset it here
      emergency_boundary_detected = true;

      // Stop all motors IMMEDIATELY
      motor_stop_all();

      // Set emergency flag
      robot_config.emergency = EMERGENCY_BOUNDARY;
      robot_config.immediate_stop_requested = true;

      // Log the emergency
      char emergency_msg[150];
      snprintf(emergency_msg, sizeof(emergency_msg), "⚠️ BOUNDARY DETECTED: %s",
               ir_sensor_get_boundary_string(boundary_data.boundaries));

      printf("%s\n", emergency_msg);

      // DO NOT reset emergency_boundary_detected here
      // Let the main task handle it
    }
    // DO NOT set emergency_boundary_detected = false here

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
static void robot_handle_boundary_escape(boundary_detection_t boundaries) {
  // Immediate stop first
  motor_stop_all();
  vTaskDelay(pdMS_TO_TICKS(50)); // Very short pause

  printf("🚨 BOUNDARY: %s\n", ir_sensor_get_boundary_string(boundaries));

  robot_config.boundary_escape_count++;
  robot_config.state = STATE_ESCAPING_BOUNDARY;

  // Use MAX speed for escape (255)
  uint8_t escape_speed = 150;

  // ONE aggressive move - no small chunks!
  if (boundaries & BOUNDARY_FRONT) {
    // Move backward LONGER, then turn MORE
    motor_backward(escape_speed);
    vTaskDelay(pdMS_TO_TICKS(1000)); // 600ms backward
    motor_stop_all();
    vTaskDelay(pdMS_TO_TICKS(30));

    // Turn MORE aggressively
    motor_spin_left(escape_speed);
    vTaskDelay(pdMS_TO_TICKS(500)); // 500ms spin
    motor_stop_all();

  } else if (boundaries & BOUNDARY_REAR) {
    motor_forward(escape_speed);
    vTaskDelay(pdMS_TO_TICKS(1000));
    motor_stop_all();
    vTaskDelay(pdMS_TO_TICKS(30));

    motor_spin_right(escape_speed);
    vTaskDelay(pdMS_TO_TICKS(500));
    motor_stop_all();

  } else if (boundaries & BOUNDARY_LEFT) {
    // Spin right HARD to escape left boundary
    motor_spin_right(escape_speed);
    vTaskDelay(pdMS_TO_TICKS(1000)); // 800ms - aggressive turn
    motor_stop_all();

  } else if (boundaries & BOUNDARY_RIGHT) {
    // Spin left HARD to escape right boundary
    motor_spin_left(escape_speed);
    vTaskDelay(pdMS_TO_TICKS(1000)); // 800ms - aggressive turn
    motor_stop_all();
  }

  // Quick pause
  vTaskDelay(pdMS_TO_TICKS(100));

  // IMMEDIATELY clear all flags and go back to searching
  robot_config.emergency = EMERGENCY_NONE;
  robot_config.immediate_stop_requested = false;
  emergency_boundary_detected = false;

  // Force state to searching - don't check again!
  robot_config.state = STATE_SEARCHING;

  printf("✅ ESCAPE COMPLETE - Back to searching\n");
}

static void robot_handle_enemy_attack(enemy_position_t enemy_pos) {
  // Check if emergency stop was requested during attack
  if (robot_config.immediate_stop_requested) {
    motor_stop_all();
    robot_config.immediate_stop_requested = false;
    return; // Abort attack immediately
  }

  char message[80];
  snprintf(message, sizeof(message), "ATTACK: %s - Engaging!",
           ultrasonic_get_enemy_position_string(enemy_pos));

  robot_config.state = STATE_ATTACKING;
  robot_config.successful_attacks++;

  // Modified attack sequence with boundary checks
  uint32_t attack_duration_ms = 0;

  switch (enemy_pos) {
  case ENEMY_FRONT:
    // Charge forward with boundary checking
    motor_forward(255);
    attack_duration_ms = 1000;
    break;

  case ENEMY_REAR:
    // Reverse attack with boundary checking
    motor_backward(255);
    attack_duration_ms = 1000;
    break;

  case ENEMY_BOTH:
    // Spin attack with boundary checking
    motor_spin_left(255);
    attack_duration_ms = 800;
    break;

  case ENEMY_NONE:
    return; // Should not happen
  }

  // Break attack into small chunks to check for boundaries
  uint32_t chunk_size = 60; // Check every 100ms
  uint32_t chunks = attack_duration_ms / chunk_size;

  for (uint32_t i = 0; i < chunks; i++) {
    // Check for emergency stop
    if (emergency_boundary_detected || robot_config.immediate_stop_requested) {
      motor_stop_all();
      robot_config.immediate_stop_requested = false;
      return; // Abort attack immediately
    }

    vTaskDelay(pdMS_TO_TICKS(chunk_size));
  }

  // Check one more time before backoff
  if (emergency_boundary_detected || robot_config.immediate_stop_requested) {
    motor_stop_all();
    robot_config.immediate_stop_requested = false;
    return;
  }

  // Back off after attack (with boundary checks)
  motor_stop();

  // Check during stop delay
  for (int i = 0; i < 3; i++) { // 3 * 100ms = 300ms
    if (emergency_boundary_detected || robot_config.immediate_stop_requested) {
      robot_config.immediate_stop_requested = false;
      return;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  // Backward movement with checks
  if (!emergency_boundary_detected && !robot_config.immediate_stop_requested) {
    motor_backward(robot_config.motor_speed);

    // Check during backward movement
    for (int i = 0; i < 4; i++) { // 4 * 100ms = 400ms
      if (emergency_boundary_detected ||
          robot_config.immediate_stop_requested) {
        motor_stop_all();
        robot_config.immediate_stop_requested = false;
        return;
      }
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
  motor_stop();
}

// new enhanced serach method
static void robot_enhanced_search(void) {
  // Smart search algorithm with memory
  robot_config.state = STATE_SEARCHING;

  // Pattern 1: If enemy was recently seen, search in that direction
  if (last_enemy_position != ENEMY_NONE &&
      (xTaskGetTickCount() * portTICK_PERIOD_MS - last_seen_time) < 5000) {

    // Biased search towards last seen position
    switch (last_enemy_position) {
    case ENEMY_FRONT:
      // Move forward with scanning
      motor_forward(robot_config.motor_speed);
      vTaskDelay(pdMS_TO_TICKS(1000));
      motor_spin_left(robot_config.motor_speed);
      vTaskDelay(pdMS_TO_TICKS(500));
      break;
    case ENEMY_REAR:
      // Turn around and search
      motor_spin_left(robot_config.motor_speed);
      vTaskDelay(pdMS_TO_TICKS(800));
      motor_forward(robot_config.motor_speed);
      vTaskDelay(pdMS_TO_TICKS(800));
      break;
    case ENEMY_BOTH:
      // Spin and search aggressively
      motor_spin_left(robot_config.motor_speed);
      vTaskDelay(pdMS_TO_TICKS(800));
      motor_forward(robot_config.motor_speed);
      vTaskDelay(pdMS_TO_TICKS(600));
      break;
    default:
      break;
    }
  }
  // Pattern 2: Systematic coverage pattern
  else {
    // Spiral search pattern
    if (search_iteration % 20 == 0) {
      // Every 20 iterations, change direction
      search_direction *= -1;
    }

    switch (search_iteration % 8) {
    case 0:
    case 1:
      // Move forward
      motor_forward(robot_config.motor_speed);
      vTaskDelay(pdMS_TO_TICKS(1000 + (search_iteration % 3) * 200));
      break;
    case 2:
      // Turn in current search direction
      if (search_direction > 0) {
        motor_turn_left(robot_config.motor_speed);
      } else {
        motor_turn_right(robot_config.motor_speed);
      }
      vTaskDelay(pdMS_TO_TICKS(300 + (search_iteration % 5) * 100));
      break;
    case 3:
    case 4:
      // Move forward with varying speed
      motor_forward(robot_config.motor_speed + (search_iteration % 3) * 20);
      vTaskDelay(pdMS_TO_TICKS(800));
      break;
    case 5:
      // Quick scan turn
      motor_spin_left(robot_config.motor_speed);
      vTaskDelay(pdMS_TO_TICKS(200));
      motor_spin_right(robot_config.motor_speed);
      vTaskDelay(pdMS_TO_TICKS(200));
      break;
    case 6:
    case 7:
      // Diagonal movement
      if (search_direction > 0) {
        motor_turn_left(robot_config.motor_speed);
      } else {
        motor_turn_right(robot_config.motor_speed);
      }
      vTaskDelay(pdMS_TO_TICKS(200));
      motor_forward(robot_config.motor_speed);
      vTaskDelay(pdMS_TO_TICKS(600));
      break;
    }
    search_iteration++;
  }

  motor_stop();
  vTaskDelay(pdMS_TO_TICKS(200));
}

static void robot_search_for_enemy(void) {
  static int search_pattern = 0;

  // Quick boundary check before moving
  boundary_data_t boundary_check;
  ir_sensor_read(&boundary_check);
  if (boundary_check.boundaries != BOUNDARY_NONE) {
    return; // Let main loop handle boundary
  }

  // Simple 3-step search pattern (move, pause, turn)
  switch (search_pattern % 3) {
  case 0:
    // Move forward
    motor_forward(robot_config.motor_speed);
    vTaskDelay(pdMS_TO_TICKS(800));
    motor_stop();
    break;

  case 1:
    // Pause and scan
    vTaskDelay(pdMS_TO_TICKS(300));
    break;

  case 2:
    // Turn to scan new area
    motor_spin_left(robot_config.motor_speed);
    vTaskDelay(pdMS_TO_TICKS(200));
    motor_stop();
    break;
  }

  search_pattern++;
  vTaskDelay(pdMS_TO_TICKS(100));
}

static void robot_auto_fight_mode(void) {
  boundary_data_t boundary_data;
  enemy_data_t enemy_data;

  while (robot_config.running) {
    // Read sensors
    ir_sensor_read(&boundary_data);
    ultrasonic_read_both(&enemy_data);

    // Check for immediate stop (from watchdog)
    if (robot_config.immediate_stop_requested || emergency_boundary_detected) {
      // Handle boundary if detected
      if (boundary_data.boundaries != BOUNDARY_NONE) {
        robot_handle_boundary_escape(boundary_data.boundaries);
      }
      // Clear flags after handling
      robot_config.immediate_stop_requested = false;
      emergency_boundary_detected = false;
      continue;
    }

    // Normal priority check
    // Priority 1: Enemy attack (if close enough)
    if (enemy_data.enemy_position != ENEMY_NONE) {
      // Check if enemy is within attack range
      bool should_attack = false;

      if (enemy_data.enemy_position == ENEMY_FRONT ||
          enemy_data.enemy_position == ENEMY_BOTH) {
        should_attack =
            enemy_data.front_distance <= robot_config.attack_threshold_cm;
      }
      if (enemy_data.enemy_position == ENEMY_REAR ||
          enemy_data.enemy_position == ENEMY_BOTH) {
        should_attack = should_attack || (enemy_data.rear_distance <=
                                          robot_config.attack_threshold_cm);
      }

      if (should_attack) {
        robot_handle_enemy_attack(enemy_data.enemy_position);
      } else {
        // Enemy too far - approach
        robot_config.state = STATE_SEARCHING;
        robot_enhanced_search();
      }
    }
    // Priority 2: Search for enemy
    else {
      robot_config.state = STATE_SEARCHING;

      // Use memory-based search if enemy was recently seen
      if (last_enemy_position != ENEMY_NONE &&
          (xTaskGetTickCount() * portTICK_PERIOD_MS - last_seen_time) < 3000) {
        robot_enhanced_search();
      } else {
        robot_search_for_enemy();
      }
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

static void robot_manual_mode(void) {
  boundary_data_t boundary_data;
  enemy_data_t enemy_data;

  while (robot_config.running) {
    // Read sensors for display only
    ir_sensor_read(&boundary_data);
    ultrasonic_read_both(&enemy_data);

    // Print sensor status in manual mode
    char status[200];
    snprintf(
        status, sizeof(status),
        "MANUAL 4WD: Boundaries: %s | Enemy: %s (F:%.1fcm R:%.1fcm) | Speed:%d",
        ir_sensor_get_boundary_string(boundary_data.boundaries),
        ultrasonic_get_enemy_position_string(enemy_data.enemy_position),
        enemy_data.front_distance, enemy_data.rear_distance,
        robot_config.motor_speed);

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

static void robot_motor_test_mode(void) {
  robot_config.state = STATE_TESTING;
  robot_run_motor_test();
  robot_config.state = STATE_IDLE;
}

static void robot_task(void *pvParameters) {

  while (1) {
    if (robot_config.running) {
      switch (robot_config.mode) {
      case MODE_AUTO_FIGHT:
        robot_auto_fight_mode();
        break;
      case MODE_MANUAL:
        robot_manual_mode();
        break;
      case MODE_MOTOR_TEST:
        robot_motor_test_mode();
        robot_config.running = false; // Stop after test
        break;
      case MODE_DEMO:
        // Demo mode implementation
        vTaskDelay(pdMS_TO_TICKS(1000));
        break;
      }
    } else {
      motor_stop();
      robot_config.state = STATE_IDLE;
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
}

void robot_controller_init(void) {
  motor_driver_init();
  ir_sensor_init();
  ultrasonic_init();

  // Initialize mutex for motor control
  motor_stop_mutex = xSemaphoreCreateMutex();

  // Start boundary watchdog task (HIGHEST PRIORITY)
  xTaskCreate(boundary_watchdog_task, "boundary_watchdog", 4096, NULL, 5,
              &boundary_watchdog_handle);

  robot_config.running = false;
  robot_config.state = STATE_IDLE;
  robot_config.emergency = EMERGENCY_NONE;
  robot_config.immediate_stop_requested = false;

  printf("Robot controller initialized with emergency boundary stop system\n");
}
void robot_controller_start(void) {
  if (robot_task_handle == NULL) {
    robot_config.running = true;
    xTaskCreate(robot_task, "robot_controller", 8192, NULL, 2,
                &robot_task_handle);
  }
}

void robot_controller_stop(void) {
  robot_config.running = false;
  motor_stop();
  if (robot_task_handle != NULL) {
    vTaskDelete(robot_task_handle);
    robot_task_handle = NULL;
  }
}

void robot_set_mode(robot_mode_t mode) {
  robot_config.mode = mode;
  char mode_str[20];
  switch (mode) {
  case MODE_MANUAL:
    strcpy(mode_str, "MANUAL");
    break;
  case MODE_AUTO_FIGHT:
    strcpy(mode_str, "AUTO FIGHT");
    break;
  case MODE_DEMO:
    strcpy(mode_str, "DEMO");
    break;
  case MODE_MOTOR_TEST:
    strcpy(mode_str, "MOTOR TEST");
    break;
  }

  char message[50];
  snprintf(message, sizeof(message), "4WD Robot mode changed to: %s", mode_str);
}

void robot_set_speed(uint8_t speed) {
  if (speed <= 255) {
    robot_config.motor_speed = speed;
    char message[40];
    snprintf(message, sizeof(message), "4WD Motor speed set to: %d", speed);
  }
}

void robot_set_attack_threshold(float threshold_cm) {
  if (threshold_cm > 0 && threshold_cm <= 30) {
    robot_config.attack_threshold_cm = threshold_cm;
    char message[50];
    snprintf(message, sizeof(message), "Attack threshold set to: %.1f cm",
             threshold_cm);
  }
}

robot_config_t robot_get_config(void) { return robot_config; }

const char *robot_get_state_string(robot_state_t state) {
  switch (state) {
  case STATE_SEARCHING:
    return "SEARCHING";
  case STATE_ATTACKING:
    return "ATTACKING";
  case STATE_ESCAPING_BOUNDARY:
    return "ESCAPING_BOUNDARY";
  case STATE_RECOVERING:
    return "RECOVERING";
  case STATE_IDLE:
    return "IDLE";
  case STATE_TESTING:
    return "TESTING";
  default:
    return "UNKNOWN";
  }
}

bool robot_is_emergency_boundary_detected(void) {
  return emergency_boundary_detected;
}

bool robot_is_immediate_stop_requested(void) {
  return robot_config.immediate_stop_requested;
}

void robot_clear_emergency_flags(void) {
  emergency_boundary_detected = false;
  robot_config.immediate_stop_requested = false;
  robot_config.emergency = EMERGENCY_NONE;
}

void robot_run_motor_test(void) { motor_test_sequence(); }
