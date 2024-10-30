#ifndef _esp32s2_data_log_h
#define _esp32s2_data_log_h

#include "esp23s2_pac1720.h"
#include "esp32s2_24lc512.h"
#include "esp32s2_adc.h"
#include "esp32s2_buff_operations.h"
#include "esp32s2_gpio.h"
#include "esp32s2_https_protocol.h"
#include "esp32s2_i2c_protocol.h"
#include "esp32s2_socket.h"
#include "esp32s2_button.h"
#include "esp32s2_wifi.h"

typedef struct logging_struct_tag{
	uint16_t log_head;
	uint16_t log_tail;
	uint16_t log_available;
}logging_struct_t;

#define LOG_START_ADDR 		0
#define LOG_MAX_RECORDS		713 
#define LOG_END_ADDR		(LOG_START_ADDR + (LOG_MAX_RECORDS * SUMMARY_LEN))
#define CONT_LOG_STOP_CNT	240 

#define EE_LOG_IDX_START_ADDR 	0xFE00
#define VALID_IDX_ADDR			(EE_LOG_IDX_START_ADDR + sizeof(log_struct))
#define VALID_IDX_VALUE			0x22334466

extern void turtle_log_flush();
extern void turtle_log_put(uint8_t *log_in_buff);
extern void turtle_log_get(uint8_t *log_in_buff,uint8_t pos);
extern void turtle_log_update_tail();
extern uint16_t turtle_log_available();
extern void log_to_eeprom();
extern void turtle_log_test();
extern void turtle_log_put_test();
extern void turtle_log_idx_restore();
extern void turtle_log_idx_save();
extern void turtle_log_idx_flush();

#endif