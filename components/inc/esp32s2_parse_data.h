#ifndef _esp32s2_parse_data_h
#define _esp32s2_parse_data_h

#include "esp32s2_main.h"
#include "string.h"
#include "cJSON.h"
#include "esp_log.h"
#include "stdint.h"

#define HTTPS_REQ_BUFFER_SIZE 10
#define parse_response(x,y) (x&y)==y? 1:0

typedef enum{
    get_information=0,
    update_calibration=1,
    update_configuration=2,
    update_system_threshold=3,
    send_response=4,
    update_useable_soc=5,
    log_data=6,
    update_certificate=7,
    read_certificate=8,
    ota_update=9,
    domain_update=10,
}https_request_commands_t;

typedef enum{
    calibration_req = 1,
    configuration_req = 2,
    system_thr_req = 4,
    useable_soc_req = 8,
    factory_update = 16,
    factory_reset = 32,
}server_read_responses_t;

struct https_request_buffer_body{
    uint8_t https_request_buffer_head;
    uint8_t https_request_buffer_tail;
    uint8_t https_request;
    uint8_t https_request_buffer[HTTPS_REQ_BUFFER_SIZE];
};

typedef struct https_request_buffer_body https_request_buffer_body_t;

extern void httpsmsgparse(char *https_receive_buffer,https_request_buffer_body_t *param);
extern void https_req_buff_write(https_request_buffer_body_t *param,uint8_t c);
extern uint8_t https_req_buff_available(https_request_buffer_body_t *param);
extern uint8_t https_req_buff_peek(https_request_buffer_body_t *param);
extern void https_req_buff_flush(https_request_buffer_body_t *param);
extern int detect_payload(cJSON *payload);
extern int detect_msgType(cJSON *payload);
extern void detect_write_msg(cJSON *payload);
extern void detect_read_msg(cJSON *payload);
extern void parse_write_msg(cJSON *writeInfo);
extern void parse_read_msg(cJSON *readInfo);
extern void test_parsing();
#endif