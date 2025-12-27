#ifndef WIFI_REMOTE_H
#define WIFI_REMOTE_H

#include "cJSON.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

typedef enum {
  WIFI_REMOTE_MODE_OFF, // Changed from REMOTE_WIFI_MODE_OFF
  WIFI_REMOTE_MODE_AP,  // Changed from REMOTE_WIFI_MODE_AP
  WIFI_REMOTE_MODE_STA  // Changed from REMOTE_WIFI_MODE_STA
} wifi_remote_mode_t;

typedef struct {
  wifi_remote_mode_t mode;
  char ssid[32];
  char password[64];
  uint16_t port;
  bool enabled;
  bool connected;
} wifi_remote_config_t;

typedef struct {
  char type[20]; // "command", "status", "sensor_data"
  char data[256];
  uint32_t timestamp;
} wifi_message_t;

// Initialize WiFi remote control
void wifi_remote_init(void);
void wifi_remote_start(void);
void wifi_remote_stop(void);
void wifi_remote_set_mode(wifi_remote_mode_t mode);
void wifi_remote_set_credentials(const char *ssid, const char *password);
void wifi_remote_send_message(const char *type, const char *data);

// Web server handlers
void wifi_start_webserver(void);
void wifi_stop_webserver(void);

// Get current status
wifi_remote_config_t wifi_remote_get_status(void);
bool wifi_remote_is_connected(void);

void process_command(char *command);
/**
 * @brief take string and execute the related function
 * valid strings -> forward,backward,


**/
#endif
