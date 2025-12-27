#include "wifi_remote.h"
#include "cJSON.h"
#include "ir_sensor.h"
#include "motor_driver.h"
#include "pin_config.h"
#include "robot_controller.h"
#include "ultrasonic.h"
#include <ctype.h>
#include <string.h>
#include <sys/param.h>

static const char *TAG = "WIFI_REMOTE";

static wifi_remote_config_t wifi_remote_config = {
    .mode = WIFI_REMOTE_MODE_AP, // Updated enum value
    .ssid = "D-Link",
    .password = "26062024",
    .port = 80,
    .enabled = false,
    .connected = false};

static httpd_handle_t server = NULL;
static QueueHandle_t wifi_tx_queue = NULL;
static TaskHandle_t wifi_task_handle = NULL;

// WiFi event handler (unchanged)
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT) {
    switch (event_id) {
    case WIFI_EVENT_STA_START:
      ESP_LOGI(TAG, "STA mode started");
      esp_wifi_connect();
      break;
    case WIFI_EVENT_STA_DISCONNECTED:
      wifi_remote_config.connected = false;
      wifi_remote_config.enabled = true; // Still enabled, just disconnected
      ESP_LOGI(TAG, "WiFi disconnected, retrying...");
      esp_wifi_connect();
      break;
    case WIFI_EVENT_AP_START:
      ESP_LOGI(TAG, "AP mode started - SSID: %s", wifi_remote_config.ssid);
      wifi_remote_config.connected = false; // No clients yet
      wifi_remote_config.enabled = true;
      break;
    case WIFI_EVENT_AP_STACONNECTED:
      wifi_remote_config.connected = true;
      ESP_LOGI(TAG, "Client connected to AP");
      // Print client MAC address
      wifi_event_ap_staconnected_t *event =
          (wifi_event_ap_staconnected_t *)event_data;
      ESP_LOGI(TAG, "Client MAC: %02x:%02x:%02x:%02x:%02x:%02x", event->mac[0],
               event->mac[1], event->mac[2], event->mac[3], event->mac[4],
               event->mac[5]);
      break;
    case WIFI_EVENT_AP_STADISCONNECTED:
      wifi_remote_config.connected = false;
      ESP_LOGI(TAG, "Client disconnected from AP");
      break;
    }
  } else if (event_base == IP_EVENT) {
    switch (event_id) {
    case IP_EVENT_STA_GOT_IP: {
      wifi_remote_config.connected = true;
      ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
      ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    } break;
    }
  }
}
// HTTP request handler for robot commands (unchanged)
static esp_err_t command_post_handler(httpd_req_t *req) {
  char buf[256];
  int ret, remaining = req->content_len;

  if (remaining > sizeof(buf) - 1) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                        "Content too long");
    return ESP_FAIL;
  }

  ret = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf) - 1));
  if (ret <= 0) {
    return ESP_FAIL;
  }
  buf[ret] = '\0';

  ESP_LOGI(TAG, "Received command: %s", buf);

  // Parse JSON command
  cJSON *root = cJSON_Parse(buf);
  if (root != NULL) {
    cJSON *command = cJSON_GetObjectItem(root, "command");
    if (cJSON_IsString(command)) {
      char cmd_str[100];
      snprintf(cmd_str, sizeof(cmd_str), "%s", command->valuestring);
      process_command(cmd_str);
      // Send response
      char response[200];
      snprintf(
          response, sizeof(response),
          "{\"status\":\"success\", \"command\":\"%s\", \"timestamp\":%lu}",
          command->valuestring, xTaskGetTickCount() * portTICK_PERIOD_MS);
      httpd_resp_set_type(req, "application/json");
      httpd_resp_send(req, response, strlen(response));
    }
    cJSON_Delete(root);
  } else {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
  }

  return ESP_OK;
}

// HTTP handler for sensor data (unchanged)
static esp_err_t sensors_get_handler(httpd_req_t *req) {
  // Get current sensor data
  boundary_data_t ir_data;
  enemy_data_t us_data;
  ir_sensor_read(&ir_data);
  ultrasonic_read_both(&us_data);
  // Create JSON response
  cJSON *root = cJSON_CreateObject();
  cJSON_AddNumberToObject(root, "timestamp",
                          xTaskGetTickCount() * portTICK_PERIOD_MS);

  // IR sensor data
  cJSON *ir_json = cJSON_CreateObject();
  cJSON_AddBoolToObject(ir_json, "front_left", ir_data.front_left);
  cJSON_AddBoolToObject(ir_json, "front_right", ir_data.front_right);
  cJSON_AddBoolToObject(ir_json, "rear_left", ir_data.rear_left);
  cJSON_AddBoolToObject(ir_json, "rear_right", ir_data.rear_right);
  cJSON_AddStringToObject(ir_json, "boundaries",
                          ir_sensor_get_boundary_string(ir_data.boundaries));
  cJSON_AddItemToObject(root, "ir_sensors", ir_json);

  // Ultrasonic data
  cJSON *us_json = cJSON_CreateObject();
  cJSON_AddNumberToObject(us_json, "front_distance", us_data.front_distance);
  cJSON_AddNumberToObject(us_json, "rear_distance", us_data.rear_distance);
  cJSON_AddStringToObject(
      us_json, "enemy_position",
      ultrasonic_get_enemy_position_string(us_data.enemy_position));
  cJSON_AddItemToObject(root, "ultrasonic", us_json);
  // Robot status
  robot_config_t robot_status = robot_get_config();
  cJSON *robot_json = cJSON_CreateObject();
  cJSON_AddStringToObject(robot_json, "state",
                          robot_get_state_string(robot_status.state));
  cJSON_AddNumberToObject(robot_json, "speed", robot_status.motor_speed);
  cJSON_AddBoolToObject(robot_json, "running", robot_status.running);
  cJSON_AddItemToObject(root, "robot", robot_json);

  char *json_str = cJSON_Print(root);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_send(req, json_str, strlen(json_str));

  free(json_str);
  cJSON_Delete(root);
  return ESP_OK;
}

// HTTP handler for robot status (unchanged)
static esp_err_t status_get_handler(httpd_req_t *req) {
  robot_config_t status = robot_get_config();

  cJSON *root = cJSON_CreateObject();
  cJSON_AddStringToObject(root, "state", robot_get_state_string(status.state));
  cJSON_AddNumberToObject(root, "mode", status.mode);
  cJSON_AddNumberToObject(root, "speed", status.motor_speed);
  cJSON_AddNumberToObject(root, "threshold", status.attack_threshold_cm);
  cJSON_AddBoolToObject(root, "running", status.running);
  cJSON_AddNumberToObject(root, "boundary_escapes",
                          status.boundary_escape_count);
  cJSON_AddNumberToObject(root, "successful_attacks",
                          status.successful_attacks);
  cJSON_AddNumberToObject(root, "timestamp",
                          xTaskGetTickCount() * portTICK_PERIOD_MS);
  cJSON_AddNumberToObject(root, "emergency", status.emergency);
  cJSON_AddBoolToObject(root, "immediate_stop",
                        status.immediate_stop_requested);
  char *json_str = cJSON_Print(root);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_send(req, json_str, strlen(json_str));
  free(json_str);
  cJSON_Delete(root);
  return ESP_OK;
}

static esp_err_t wifi_status_get_handler(httpd_req_t *req) {
  wifi_remote_config_t wifi_status = wifi_remote_get_status();
  // Get IP address if connected
  esp_netif_ip_info_t ip_info;
  char ip_str[16] = "N/A";

  if (wifi_status.connected) {
    if (wifi_status.mode == WIFI_REMOTE_MODE_AP) {
      esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_AP_DEF"),
                            &ip_info);
      snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&ip_info.ip));
    } else {
      esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"),
                            &ip_info);
      snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&ip_info.ip));
    }
  }

  cJSON *root = cJSON_CreateObject();
  cJSON_AddNumberToObject(root, "mode", wifi_status.mode);
  cJSON_AddStringToObject(root, "ssid", wifi_status.ssid);
  cJSON_AddBoolToObject(root, "enabled", wifi_status.enabled);
  cJSON_AddBoolToObject(root, "connected", wifi_status.connected);
  cJSON_AddStringToObject(root, "ip_address", ip_str);
  cJSON_AddNumberToObject(root, "port", wifi_status.port);
  cJSON_AddNumberToObject(root, "timestamp",
                          xTaskGetTickCount() * portTICK_PERIOD_MS);

  char *json_str = cJSON_Print(root);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_send(req, json_str, strlen(json_str));

  free(json_str);
  cJSON_Delete(root);
  return ESP_OK;
}

// Replace root_get_handler in wifi_remote.c with this MINIMAL version:
// Replace root_get_handler in wifi_remote.c with this MINIMAL version:
static esp_err_t root_get_handler(httpd_req_t *req) {
  const char *html_page =
      "<!DOCTYPE html>"
      "<html>"
      "<head>"
      "<title>Robot Control</title>"
      "<meta name='viewport' content='width=device-width, initial-scale=1'>"
      "<style>"
      "body{font-family:Arial;margin:20px;background:#f0f0f0;}"
      ".container{max-width:800px;margin:auto;background:white;padding:20px;"
      "border-radius:10px;}"
      ".btn{padding:15px;margin:5px;border:none;border-radius:5px;color:white;"
      "cursor:pointer;font-size:16px;}"
      ".green{background:#4CAF50;}.red{background:#f44336;}.blue{background:#"
      "2196F3;}.orange{background:#FF9800;}"
      ".status{padding:10px;background:#e0e0e0;border-radius:5px;margin:10px "
      "0;}"
      ".data{font-family:monospace;background:#333;color:#0f0;padding:10px;"
      "border-radius:5px;}"
      "</style>"
      "</head>"
      "<body>"
      "<div class='container'>"
      "<h1>Robot Control</h1>"
      "<div>"
      "<button class='btn green' onclick=\"cmd('forward')\">Forward</button>"
      "<button class='btn red' "
      "onclick=\"cmd('backward')\">Backward</button><br>"
      "<button class='btn blue' onclick=\"cmd('left')\">Left</button>"
      "<button class='btn red' onclick=\"cmd('stop')\">Stop</button>"
      "<button class='btn blue' onclick=\"cmd('right')\">Right</button><br>"
      "<button class='btn orange' onclick=\"cmd('spinleft')\">Spin L</button>"
      "<button class='btn orange' onclick=\"cmd('spinright')\">Spin R</button>"
      "</div>"
      "<div>"
      "<button class='btn' onclick=\"cmd('auto')\">Auto Mode</button>"
      "<button class='btn' onclick=\"cmd('manual')\">Manual Mode</button>"
      "<button class='btn' onclick=\"cmd('start')\">Start</button>"
      "<button class='btn red' onclick=\"cmd('stoprobot')\">Stop Robot</button>"
      "</div>"
      "<div class='status'>"
      "<h3>Status:</h3>"
      "<div id='status'>Loading...</div>"
      "</div>"
      "<div class='data'>"
      "<h3>Sensor Data:</h3>"
      "<div id='sensors'>Loading...</div>"
      "</div>"
      "<div style='margin-top:20px;'>"
      "<label>Speed: <input type='range' id='speed' min='50' max='255' "
      "value='200' onchange=\"cmd('speed '+this.value)\"></label>"
      "<br>"
      "<label>Threshold: <input type='range' id='threshold' min='10' max='100' "
      "value='30' step='0.5' onchange=\"cmd('threshold '+this.value)\"></label>"
      "</div>"
      "</div>"
      "<script>"
      "async function cmd(c){"
      "  try{"
      "    await "
      "fetch('/command',{method:'POST',headers:{'Content-Type':'application/"
      "json'},body:JSON.stringify({command:c})});"
      "    update();"
      "  }catch(e){console.error(e);}"
      "}"
      "async function update(){"
      "  try{"
      "    let r=await fetch('/status');"
      "    let d=await r.json();"
      "    document.getElementById('status').innerHTML=`State: "
      "${d.state}<br>Mode: ${d.mode}<br>Speed: ${d.speed}<br>Running: "
      "${d.running}`;"
      "    r=await fetch('/sensors');"
      "    d=await r.json();"
      "    document.getElementById('sensors').innerHTML=`Front: "
      "${d.ultrasonic.front_distance}cm<br>Rear: "
      "${d.ultrasonic.rear_distance}cm<br>Enemy: "
      "${d.ultrasonic.enemy_position}<br>Boundary: ${d.ir_sensors.boundaries}`;"
      "  }catch(e){console.error(e);}"
      "}"
      "setInterval(update,1000);"
      "update();"
      "</script>"
      "</body>"
      "</html>";

  httpd_resp_set_type(req, "text/html");
  httpd_resp_send(req, html_page, strlen(html_page));
  return ESP_OK;
}
// Define HTTP URI handlers (unchanged)
static const httpd_uri_t command_post = {.uri = "/command",
                                         .method = HTTP_POST,
                                         .handler = command_post_handler,
                                         .user_ctx = NULL};

static const httpd_uri_t sensors_get = {.uri = "/sensors",
                                        .method = HTTP_GET,
                                        .handler = sensors_get_handler,
                                        .user_ctx = NULL};

static const httpd_uri_t status_get = {.uri = "/status",
                                       .method = HTTP_GET,
                                       .handler = status_get_handler,
                                       .user_ctx = NULL};

static const httpd_uri_t root_get = {.uri = "/",
                                     .method = HTTP_GET,
                                     .handler = root_get_handler,
                                     .user_ctx = NULL};
// Add to wifi_start_webserver function:
static const httpd_uri_t wifi_status_get = {.uri = "/wifi_status",
                                            .method = HTTP_GET,
                                            .handler = wifi_status_get_handler,
                                            .user_ctx = NULL};

void wifi_start_webserver(void) {
  if (server != NULL) {
    return;
  }

  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = wifi_remote_config.port;

  if (httpd_start(&server, &config) == ESP_OK) {
    httpd_register_uri_handler(server, &command_post);
    httpd_register_uri_handler(server, &sensors_get);
    httpd_register_uri_handler(server, &status_get);
    httpd_register_uri_handler(server, &root_get);
    httpd_register_uri_handler(server, &wifi_status_get);
    ESP_LOGI(TAG, "Web server started on port %d", config.server_port);
  }
}

void wifi_stop_webserver(void) {
  if (server != NULL) {
    httpd_stop(server);
    server = NULL;
    ESP_LOGI(TAG, "Web server stopped");
  }
}

static void wifi_init_ap(void) {
  wifi_config_t wifi_config_ap = {
      .ap = {.ssid_len = strlen(wifi_remote_config.ssid),
             .channel = 1,
             .max_connection = 4,
             .authmode = WIFI_AUTH_WPA_WPA2_PSK},
  };

  strcpy((char *)wifi_config_ap.ap.ssid, wifi_remote_config.ssid);
  strcpy((char *)wifi_config_ap.ap.password, wifi_remote_config.password);

  if (strlen(wifi_remote_config.password) == 0) {
    wifi_config_ap.ap.authmode = WIFI_AUTH_OPEN;
  }

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config_ap));
}

static void wifi_init_sta(void) {
  wifi_config_t wifi_config_sta = {
      .sta =
          {
              .threshold.authmode = WIFI_AUTH_WPA2_PSK,
          },
  };

  strcpy((char *)wifi_config_sta.sta.ssid, wifi_remote_config.ssid);
  strcpy((char *)wifi_config_sta.sta.password, wifi_remote_config.password);

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config_sta));
}

static void wifi_task(void *pvParameters) {
  ESP_LOGI(TAG, "WiFi task started");

  while (1) {
    if (wifi_remote_config.enabled) {
      if (!wifi_remote_config.connected) {
        ESP_LOGI(TAG, "WiFi enabled but not connected. Starting in %s mode...",
                 wifi_remote_config.mode == WIFI_REMOTE_MODE_AP ? "AP" : "STA");

        if (wifi_remote_config.mode == WIFI_REMOTE_MODE_AP) {
          wifi_init_ap();
        } else {
          wifi_init_sta();
        }

        ESP_ERROR_CHECK(esp_wifi_start());
        ESP_LOGI(TAG,
                 "WiFi started. Waiting up to 30 seconds for connection...");

        // Wait for connection
        int timeout = 30;
        while (timeout-- > 0 && !wifi_remote_config.connected) {
          vTaskDelay(pdMS_TO_TICKS(1000));
          ESP_LOGI(TAG, "Waiting... %d seconds", timeout);
        }

        if (wifi_remote_config.connected) {
          ESP_LOGI(TAG, "WiFi connected successfully!");
          wifi_start_webserver();
          ESP_LOGI(TAG, "Web server started on port %d",
                   wifi_remote_config.port);
        } else {
          ESP_LOGW(TAG, "WiFi connection timeout after 30 seconds");
          esp_wifi_stop();
          wifi_remote_config.enabled = false;
        }
      }
      // If connected, do nothing (just maintain connection)
    } else {
      // WiFi is disabled
      if (wifi_remote_config.connected) {
        ESP_LOGI(TAG, "WiFi disabled - stopping");
        wifi_stop_webserver();
        esp_wifi_stop();
        wifi_remote_config.connected = false;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
void wifi_remote_init(void) {
  wifi_tx_queue = xQueueCreate(10, sizeof(wifi_message_t));

  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
  esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
  assert(ap_netif && sta_netif);

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

  ESP_LOGI(TAG, "WiFi remote control initialized");
}

void wifi_remote_start(void) {
  if (wifi_task_handle == NULL) {
    wifi_remote_config.enabled = true;
    xTaskCreate(wifi_task, "wifi_remote", 12288, NULL, 2, &wifi_task_handle);
  }
}

void wifi_remote_stop(void) {
  wifi_remote_config.enabled = false;
  if (wifi_task_handle != NULL) {
    vTaskDelete(wifi_task_handle);
    wifi_task_handle = NULL;
  }
}

void wifi_remote_set_mode(wifi_remote_mode_t mode) {
  wifi_remote_config.mode = mode;
  char mode_str[20];
  switch (mode) {
  case WIFI_REMOTE_MODE_AP:
    strcpy(mode_str, "ACCESS POINT");
    break;
  case WIFI_REMOTE_MODE_STA:
    strcpy(mode_str, "STATION");
    break;
  case WIFI_REMOTE_MODE_OFF:
    strcpy(mode_str, "OFF");
    break;
  }

  char msg[50];
  snprintf(msg, sizeof(msg), "WiFi mode set to: %s", mode_str);
}

void wifi_remote_set_credentials(const char *ssid, const char *password) {
  strncpy(wifi_remote_config.ssid, ssid, sizeof(wifi_remote_config.ssid) - 1);
  strncpy(wifi_remote_config.password, password,
          sizeof(wifi_remote_config.password) - 1);

  char msg[100];
  snprintf(msg, sizeof(msg), "WiFi credentials updated: SSID=%s", ssid);
}

void wifi_remote_send_message(const char *type, const char *data) {
  if (wifi_remote_config.enabled && wifi_remote_config.connected) {
    wifi_message_t msg;
    strncpy(msg.type, type, sizeof(msg.type) - 1);
    strncpy(msg.data, data, sizeof(msg.data) - 1);
    msg.timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // In a real implementation, you'd send this via WebSocket
    // For now, we'll just log it
    ESP_LOGI(TAG, "WiFi Message [%s]: %s", type, data);
  }
}

wifi_remote_config_t wifi_remote_get_status(void) { return wifi_remote_config; }

bool wifi_remote_is_connected(void) { return wifi_remote_config.connected; }

// Replace the entire process_command function in wifi_remote.c:
void process_command(char *command) {
  char cmd_copy[256];
  strncpy(cmd_copy, command, sizeof(cmd_copy) - 1);
  cmd_copy[sizeof(cmd_copy) - 1] = '\0';

  // Tokenize the command
  char *token = strtok(cmd_copy, " ");
  if (!token)
    return;

  // Convert first token to lowercase
  for (int i = 0; token[i]; i++) {
    token[i] = tolower(token[i]);
  }

  int value;
  char param1[64], param2[64];

  // Motor control commands
  if (strcmp(token, "forward") == 0 || strcmp(token, "f") == 0) {
    motor_forward(200);
  } else if (strcmp(token, "backward") == 0 || strcmp(token, "b") == 0) {
    motor_backward(200);
  } else if (strcmp(token, "left") == 0 || strcmp(token, "l") == 0) {
    motor_turn_left(200);
  } else if (strcmp(token, "right") == 0 || strcmp(token, "r") == 0) {
    motor_turn_right(200);
  } else if (strcmp(token, "spinleft") == 0) {
    motor_spin_left(200);
  } else if (strcmp(token, "spinright") == 0) {
    motor_spin_right(200);
  } else if (strcmp(token, "stop") == 0 || strcmp(token, "s") == 0) {
    motor_stop();
  }
  // Mode commands
  else if (strcmp(token, "auto") == 0) {
    robot_set_mode(MODE_AUTO_FIGHT);
  } else if (strcmp(token, "manual") == 0) {
    robot_set_mode(MODE_MANUAL);
  } else if (strcmp(token, "demo") == 0) {
    robot_set_mode(MODE_DEMO);
  } else if (strcmp(token, "test") == 0) {
    robot_set_mode(MODE_MOTOR_TEST);
  }
  // Robot control
  else if (strcmp(token, "start") == 0) {
    robot_controller_start();
  } else if (strcmp(token, "stoprobot") == 0) { // Changed from "stop robot"
    robot_controller_stop();
  }
  // WiFi commands
  else if (strcmp(token, "wifion") == 0) {
    wifi_remote_start();
  } else if (strcmp(token, "wifioff") == 0) {
    wifi_remote_stop();
  } else if (strcmp(token, "wifiap") == 0) {
    wifi_remote_set_mode(WIFI_REMOTE_MODE_AP);
    wifi_remote_start();
  } else if (strcmp(token, "wifista") == 0) {
    char *ssid = strtok(NULL, " ");
    char *password = strtok(NULL, " ");
    if (ssid && password) {
      wifi_remote_set_mode(WIFI_REMOTE_MODE_STA);
      wifi_remote_set_credentials(ssid, password);
      wifi_remote_start();
    }
  }
  // Configuration commands
  else if (sscanf(command, "speed %d", &value) == 1) {
    robot_set_speed(value);
  } else if (sscanf(command, "threshold %d", &value) == 1) {
    robot_set_attack_threshold((float)value);
  } else if (strcmp(token, "status") == 0) {
    // Return current status via JSON
    char response[512];
    robot_config_t config = robot_get_config();
    wifi_remote_config_t wifi_config = wifi_remote_get_status();

    snprintf(
        response, sizeof(response),
        "{\"robot_state\":\"%s\",\"mode\":\"%d\",\"speed\":%d,\"threshold\":%."
        "1f,"
        "\"running\":%s,\"escapes\":%lu,\"attacks\":%lu,\"wifi_connected\":%s}",
        robot_get_state_string(config.state), config.mode, config.motor_speed,
        config.attack_threshold_cm, config.running ? "true" : "false",
        config.boundary_escape_count, config.successful_attacks,
        wifi_config.connected ? "true" : "false");
  }
}
