#ifndef _esp32_wifi_h_
#define _esp32_wifi_h_

#include "stdio.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "cJSON.h"
#include "string.h"
#include "esp_sntp.h"
#include "time.h"
#include "esp32s2_socket.h"

typedef enum{
    ap_mode,
    sta_mode,
}wifi_config_types;

#define primary 1
#define secondary 2

extern time_t current_time;
extern xSemaphoreHandle connectionSemaphore;
extern xSemaphoreHandle serverSemaphore;
extern xSemaphoreHandle logdataSemaphore;
extern xSemaphoreHandle testConnectionSemaphore;
extern uint8_t test_connect_cnt,connection_status_obtained;

void connectSTA(char *ssid, char *pass);
void connectAP();
void esp32s2_WiFi_Init(void);
void SyncTime_callback();
void getRTCtime();
void initRTC(void);
void stop_dhcp_server();
void start_dhcp_server();
void load_wifi_param();

#endif