#include "ultrasonic.h"
#include "esp_intr_alloc.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "pin_config.h"
#include <stdio.h>
#include <string.h>

static ultrasonic_sensor_t front_sensor;
static ultrasonic_sensor_t rear_sensor;
static bool isr_installed = false;

// IRAM_ATTR puts the ISR in IRAM for faster execution
static void IRAM_ATTR ultrasonic_echo_isr_handler(void *arg) {
  ultrasonic_sensor_t *sensor = (ultrasonic_sensor_t *)arg;
  uint32_t level = gpio_get_level(sensor->echo_pin);

  if (level == 1) {
    // Rising edge - start of pulse
    sensor->pulse_start = esp_timer_get_time();
  } else {
    // Falling edge - end of pulse
    sensor->pulse_end = esp_timer_get_time();
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(sensor->pulse_sem, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
      portYIELD_FROM_ISR();
    }
  }
}

void ultrasonic_sensor_init(ultrasonic_sensor_t *sensor, gpio_num_t trig,
                            gpio_num_t echo, uint32_t timeout) {
  sensor->trigger_pin = trig;
  sensor->echo_pin = echo;
  sensor->timeout_us = timeout;

  // Create binary semaphore for pulse measurement
  sensor->pulse_sem = xSemaphoreCreateBinary();
  sensor->pulse_start = 0;
  sensor->pulse_end = 0;

  // Configure trigger pin as output
  gpio_config_t trig_conf = {
      .pin_bit_mask = (1ULL << trig),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_config(&trig_conf);

  // Configure echo pin as input with interrupt on both edges
  gpio_config_t echo_conf = {
      .pin_bit_mask = (1ULL << echo),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_ENABLE, // Pull down to avoid floating
      .intr_type =
          GPIO_INTR_ANYEDGE, // Interrupt on both rising and falling edges
  };
  gpio_config(&echo_conf);

  // Install GPIO ISR service if not already installed
  if (!isr_installed) {
    gpio_install_isr_service(0);
    isr_installed = true;
  }

  // Add ISR handler for this echo pin
  gpio_isr_handler_add(echo, ultrasonic_echo_isr_handler, sensor);

  // Initialize trigger to low
  gpio_set_level(trig, 0);
}

float ultrasonic_measure_single(ultrasonic_sensor_t *sensor) {
  // Reset semaphore state
  xSemaphoreTake(sensor->pulse_sem, 0);

  sensor->pulse_start = 0;
  sensor->pulse_end = 0;

  // Ensure trigger is low, then send 10µs pulse
  gpio_set_level(sensor->trigger_pin, 0);
  esp_rom_delay_us(5);
  gpio_set_level(sensor->trigger_pin, 1);
  esp_rom_delay_us(10);
  gpio_set_level(sensor->trigger_pin, 0);

  // Wait for pulse measurement with timeout
  if (xSemaphoreTake(sensor->pulse_sem,
                     pdMS_TO_TICKS(sensor->timeout_us / 1000)) == pdTRUE) {
    uint64_t duration = sensor->pulse_end - sensor->pulse_start;

    // Validate pulse duration
    if (duration == 0 || duration > sensor->timeout_us) {
      return -1.0f;
    }

    // Calculate distance in cm (speed of sound = 343 m/s = 0.0343 cm/µs)
    float distance_cm = (float)duration * 0.0343f / 2.0f;

    // Validate distance range (2cm to 400cm)
    if (distance_cm < 2.0f || distance_cm > 400.0f) {
      return -1.0f;
    }

    return distance_cm;
  }

  return -1.0f; // Timeout
}

void ultrasonic_init(void) {
  // Initialize front sensor (GPIO 23 trigger, GPIO 22 echo)
  ultrasonic_sensor_init(&front_sensor, US_FRONT_TRIG, US_FRONT_ECHO, 30000);

  // Initialize rear sensor (GPIO 19 trigger, GPIO 18 echo)
  ultrasonic_sensor_init(&rear_sensor, US_REAR_TRIG, US_REAR_ECHO, 30000);

  printf("Ultrasonic sensors initialized with interrupt mode\n");
  printf("Front: TRIG=GPIO%d, ECHO=GPIO%d\n", US_FRONT_TRIG, US_FRONT_ECHO);
  printf("Rear:  TRIG=GPIO%d, ECHO=GPIO%d\n", US_REAR_TRIG, US_REAR_ECHO);
}

void ultrasonic_read_both(enemy_data_t *data) {
  // Read both sensors
  data->front_distance = ultrasonic_measure_single(&front_sensor);
  esp_rom_delay_us(100); // Small delay between readings
  data->rear_distance = ultrasonic_measure_single(&rear_sensor);
  data->timestamp = esp_timer_get_time() / 1000;

  // Determine enemy position
  bool front_enemy = (data->front_distance > 0 && data->front_distance < 80);
  bool rear_enemy = (data->rear_distance > 0 && data->rear_distance < 80);

  if (front_enemy && rear_enemy) {
    data->enemy_position = ENEMY_BOTH;
  } else if (front_enemy) {
    data->enemy_position = ENEMY_FRONT;
  } else if (rear_enemy) {
    data->enemy_position = ENEMY_REAR;
  } else {
    data->enemy_position = ENEMY_NONE;
  }
}

enemy_position_t ultrasonic_detect_enemy(float attack_threshold_cm) {
  enemy_data_t data;
  ultrasonic_read_both(&data);

  bool front_attack =
      (data.front_distance > 0 && data.front_distance <= attack_threshold_cm);
  bool rear_attack =
      (data.rear_distance > 0 && data.rear_distance <= attack_threshold_cm);

  if (front_attack && rear_attack)
    return ENEMY_BOTH;
  if (front_attack)
    return ENEMY_FRONT;
  if (rear_attack)
    return ENEMY_REAR;
  return ENEMY_NONE;
}

bool ultrasonic_is_enemy_in_range(float threshold_cm) {
  enemy_position_t enemy = ultrasonic_detect_enemy(threshold_cm);
  return enemy != ENEMY_NONE;
}

const char *ultrasonic_get_enemy_position_string(enemy_position_t position) {
  switch (position) {
  case ENEMY_NONE:
    return "NO ENEMY";
  case ENEMY_FRONT:
    return "ENEMY FRONT";
  case ENEMY_REAR:
    return "ENEMY REAR";
  case ENEMY_BOTH:
    return "ENEMY BOTH SIDES";
  default:
    return "UNKNOWN";
  }
}

void ultrasonic_print_data(enemy_data_t *data) {
  printf("ULTRASONIC: %s | Front: %.1fcm | Rear: %.1fcm | Time: %lums\n",
         ultrasonic_get_enemy_position_string(data->enemy_position),
         data->front_distance, data->rear_distance, data->timestamp);
}
