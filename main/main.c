#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "robot_controller.h"
#include "wifi_remote.h"
#include <inttypes.h>
#include <stdio.h>
void print_reset_reason(void) {
  switch (esp_reset_reason()) {
  case ESP_RST_POWERON:
    printf("Power on\n");
    break;
  case ESP_RST_SW:
    printf("Software\n");
    break;
  case ESP_RST_PANIC:
    printf("Panic!\n");
    break;
  case ESP_RST_INT_WDT:
    printf("Interrupt WDT\n");
    break;
  case ESP_RST_TASK_WDT:
    printf("Task WDT\n");
    break;
  case ESP_RST_BROWNOUT:
    printf("Brownout!\n");
    break;
  default:
    printf("Other\n");
    break;
  }
}

void app_main(void) {

  vTaskDelay(pdMS_TO_TICKS(5000)); // Wait for serial

  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    printf("Erasing NVS...\n");
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
  printf("NVS initialized\n");

  // Initialize components
  printf("Initializing WiFi...\n");
  wifi_remote_init();

  printf("Initializing Robot...\n");
  robot_controller_init();

  // Start WiFi in AP mode
  printf("Starting WiFi AP...\n");
  wifi_remote_set_mode(WIFI_REMOTE_MODE_AP);
  wifi_remote_start();

  printf("Starting Robot Controller...\n");
  robot_controller_start();

  // Main loop - simple
  int counter = 0;
  while (1) {
    if (counter % 10 == 0) { // Every 10 seconds
      printf(".");
    }
    counter++;
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
