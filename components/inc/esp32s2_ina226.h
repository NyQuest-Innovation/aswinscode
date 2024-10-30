#ifndef _esp32s2_ina226_h_
#define _esp32s2_ina226_h_

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_types.h"
#include "esp_log.h"

#define INA226_BAT_SENSE_ADDR	0x82  // Bat V-I
#define INA226_SOL_SENSE_ADDR	0x8A  // Solar V-I

#define INA226_REG_CONFIG_ADDR           (0x00)
#define INA226_REG_SHUNTVOLTAGE_ADDR     (0x01)
#define INA226_REG_BUSVOLTAGE_ADDR       (0x02)
#define INA226_REG_POWER_ADDR            (0x03)
#define INA226_REG_CURRENT_ADDR          (0x04)
#define INA226_REG_CALIBRATION_ADDR      (0x05)
#define INA226_REG_MASKENABLE_ADDR       (0x06)
#define INA226_REG_ALERTLIMIT_ADDR       (0x07) 

#define INA226_AVERAGES_1 		(0x00 << 9)
#define INA226_AVERAGES_4		(0x01 << 9)
#define INA226_AVERAGES_16		(0x02 << 9)
#define INA226_AVERAGES_64		(0x03 << 9)
#define INA226_AVERAGES_128		(0x04 << 9)
#define INA226_AVERAGES_256		(0x05 << 9)
#define INA226_AVERAGES_512		(0x06 << 9)
#define INA226_AVERAGES_1024	(0x07 << 9)

#define INA226_BUS_CONV_TIME_140US 		(0x00 << 6)
#define INA226_BUS_CONV_TIME_204US 		(0x01 << 6)
#define INA226_BUS_CONV_TIME_332US 		(0x02 << 6)
#define INA226_BUS_CONV_TIME_588US 		(0x03 << 6)
#define INA226_BUS_CONV_TIME_1100US 	(0x04 << 6)
#define INA226_BUS_CONV_TIME_2116US 	(0x05 << 6)
#define INA226_BUS_CONV_TIME_4156US 	(0x06 << 6)
#define INA226_BUS_CONV_TIME_8244US 	(0x07 << 6)

#define INA226_SHUNT_CONV_TIME_140US	(0x00 << 3)
#define INA226_SHUNT_CONV_TIME_204US	(0x01 << 3)
#define INA226_SHUNT_CONV_TIME_332US	(0x02 << 3)
#define INA226_SHUNT_CONV_TIME_588US	(0x03 << 3)
#define INA226_SHUNT_CONV_TIME_1100US 	(0x04 << 3)
#define INA226_SHUNT_CONV_TIME_2116US 	(0x05 << 3)
#define INA226_SHUNT_CONV_TIME_4156US 	(0x06 << 3)
#define INA226_SHUNT_CONV_TIME_8244US 	(0x07 << 3)

#define INA226_MODE_POWER_DOWN			0x00
#define INA226_MODE_SHUNT_TRIG 			0x01
#define INA226_MODE_BUS_TRIG 			0x02
#define INA226_MODE_SHUNT_BUS_TRIG 		0x03
#define INA226_MODE_ADC_OFF 			0x04
#define INA226_MODE_SHUNT_CONT 			0x05
#define INA226_MODE_BUS_CONT 			0x06
#define INA226_MODE_SHUNT_BUS_CONT		0x07



extern void select_i2c_ina226(uint8_t i2c_slave_ina226_addr);
extern int8_t  ina226_write_reg(uint8_t reg_addr, uint16_t reg_val);
extern  int8_t  ina226_read_reg(uint8_t reg_addr, int16_t *reg_val);
extern  void test_ina226();
extern esp_err_t ina226_busy();
extern void init_ina226(uint16_t bat_i_cal, uint16_t sol_i_cal);

#endif