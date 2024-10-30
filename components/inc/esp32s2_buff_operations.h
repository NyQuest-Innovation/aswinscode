#ifndef _esp32s2_buff_operation_
#define _esp32s2_buff_operation_

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

typedef enum{
    STREAM_TIMEOUT_LONG	= 5000, 
    STREAM_TIMEOUT_NORMAL	= 1000,
    STREAM_TIMEOUT_SHORT	= 100, 
}buff_read_timeout;

uint32_t time_milli(void);
void set_uart_timeout(uint32_t _uart_time_out);
void debug_buff_write(uint8_t c);
uint8_t debug_buff_available();
uint8_t debug_buff_read();
uint8_t debug_buff_timed_read(uint8_t *c);
uint8_t debug_buff_peek();
uint8_t debug_buff_timed_peek(uint8_t *c);
void debug_buff_flush();
uint8_t debug_buff_read_line(uint8_t *rx_buff);
uint8_t debug_port_search(const uint8_t *search_str);

#define test_port_timed_read debug_buff_timed_read
#endif