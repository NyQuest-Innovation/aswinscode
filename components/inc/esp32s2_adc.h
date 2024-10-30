#ifndef _esp32s2_adc_h
#define _esp32s2_adc_h

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp32s2_socket.h"

extern float calc_bat_v, calc_bat_chg_i, calc_bat_dis_i;
extern float calc_sol_v, calc_sol_i;

#define FLUSH_ADC_VAL	1
#define MV_AVG_ADC_VAL	0
#define ADC_MUL_DIV_FACT		3 //(2 ^ 3 = 8)

typedef enum{
    ADC_INV_V  = ADC1_CHANNEL_2, 
    ADC_SOL_V  = ADC1_CHANNEL_3, 
    ADC_LOAD_I = ADC1_CHANNEL_4, 
    ADC_TEMP_1 = ADC1_CHANNEL_0, 
    ADC_V_REF  = ADC1_CHANNEL_6, 
    ADC_TEMP_2 = ADC1_CHANNEL_1,
    ADC_MAINS  = ADC1_CHANNEL_5,
}adc_channel_id;



extern void init_adc();
extern uint16_t get_adc(uint8_t ch_no);
extern void get_uc_adc_all(uint8_t *adc_buff);
extern int8_t get_sensor_values(uint8_t is_flush, uint8_t is_tx);
extern uint16_t get_adc_avg(uint8_t _adc_ch, uint8_t num_samples);
extern int8_t read_adc_all(uint8_t *adc_all_buff);
extern float get_bat_vol();

#endif