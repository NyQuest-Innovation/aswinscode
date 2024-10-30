#ifndef _esp32s2_https_protocol_
#define _esp32s2_https_protocol_

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_log.h"
#include "esp_system.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"

#include "cJSON.h"
#include "driver/gpio.h"
#include "esp32s2_algorithm.h"
#include "esp32s2_parse_data.h"
#include "esp32s2_https_protocol.h"

#include "nvs_flash.h"

#define atoh(x) (int)strtol(x,NULL,16)

typedef struct{
    char *key;
    char *val;
} Header;

typedef enum{
    Get,
    Post
} HttpMethod;

struct FetchParms{
    void (*OnGotData)(char *incomingBuffer, char *output);
    char message[300];
    Header header[3];
    int headerCount;
    HttpMethod method;
    char *body;
    int status;
};


#define RECORDS_PER_PACKET			1
#define MIN_RCORDS_TO_SERV			20
#define AUTH_DAT_LEN				21 // without header and check sum
#define LOG_DAT_LEN 				(SUMMARY_LEN * RECORDS_PER_PACKET)

extern struct FetchParms smsStruct;
extern void fetch(char *url, struct FetchParms *fetchParms);
extern void prepare_LOGDATA(char *out,uint8_t cond);
extern void send_DATA_to_server(void);
extern void test_server_connection(void);
extern void get_read_write_DATA_from_server();
extern void bidirectional_communication_commands(https_request_buffer_body_t *param);

#endif