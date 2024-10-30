#ifndef _esp32s2_24lc512_
#define _esp32s2_24lc512_

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_types.h"

#define MEM_PAGE_LEN	128

void select_i2c_memory(uint8_t i2c_memory_addr);
esp_err_t ee24lc256_busy(void);
uint8_t ee24lc256_read_byte(uint16_t memory_location);
esp_err_t i2c_master_write_slave(uint8_t *data_wr, size_t size);
esp_err_t ee24lc256_write_byte(uint16_t mem_addr, uint8_t mem_data);
int8_t ee24lc256_read_buff(uint16_t mem_addr, uint8_t *mem_buff, uint8_t mem_len);
esp_err_t ee24lc256_write_page(uint16_t mem_addr, uint8_t *mem_buff, uint8_t mem_len);
esp_err_t ee24lc256_write_buff(uint16_t mem_addr, uint8_t *mem_buff, uint8_t mem_len);
void ee24lc256_write_int16(uint16_t mem_addr, uint16_t int16_dat);
uint16_t ee24lc256_read_int16(uint16_t mem_addr);
void ee24lc256_write_int32(uint16_t mem_addr, uint32_t int32_dat);
uint32_t ee24lc256_read_int32(uint16_t mem_addr);
void ee24lc256_write_float(uint16_t mem_addr, float float_dat);
float ee24lc256_read_float(uint16_t mem_addr);
uint8_t check_memory();

#define i2c_eeprom_write_byte	ee24lc256_write_byte
#define i2c_eeprom_read_byte 	ee24lc256_read_byte
#define i2c_eeprom_busy 		ee24lc256_busy
#define i2c_eeprom_write_page 	ee24lc256_write_page
#define i2c_eeprom_read_buff 	ee24lc256_read_buff
#define i2c_eeprom_write_buff 	ee24lc256_write_buff
#define i2c_eeprom_write_int16 	ee24lc256_write_int16
#define i2c_eeprom_write_int32 	ee24lc256_write_int32
#define i2c_eeprom_read_int16 	ee24lc256_read_int16
#define i2c_eeprom_read_int32 	ee24lc256_read_int32
#define i2c_eeprom_write_float	ee24lc256_write_float
#define i2c_eeprom_read_float 	ee24lc256_read_float

#endif