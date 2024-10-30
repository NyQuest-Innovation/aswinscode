#include "esp32s2_adc.h"
#include "esp32s2_pac1952.h"

#define TAG "ADC"

const uint8_t adc_ch_array[6] = {ADC_INV_V, ADC_SOL_V, ADC_LOAD_I, ADC_TEMP_1, ADC_TEMP_2, ADC_TEMP_2};

float inverter_vol=0.0,temperature1=0.0,temperature2=0.0,vol_ref=0.0;
uint8_t mains=0;

extern uint8_t i2cFlag;

void init_adc(void){
   adc1_config_width(ADC_WIDTH_BIT_13);
   for(int i=0;i<6;i++){
	adc1_config_channel_atten(adc_ch_array[i], ADC_ATTEN_DB_11);   
   }
}

void get_uc_adc_all(uint8_t *adc_buff){
	uint8_t adc_count, buff_idx;
	uint16_t adc_val;
	for(adc_count = 0, buff_idx = 0; adc_count < 6; adc_count++, buff_idx += 2){
		adc_val = adc1_get_raw(adc_ch_array[adc_count]);
		adc_buff[buff_idx] = ((adc_val >> 8) & 0xFF);
		adc_buff[buff_idx + 1] = ( adc_val & 0xFF);
		vTaskDelay(10/portTICK_PERIOD_MS);
	}
}

float get_bat_vol(){
	while(i2cFlag){
		ESP_LOGI(TAG,"I2C bus busy");
	};
	vTaskDelay(10/portTICK_RATE_MS);
	i2cFlag=1;
	uint16_t batValue;
	float batVoltage;
	PAC194X5X_getBatteryVoltage(pPACdevice,&batValue);
	batVoltage = pac_bat_v_const * batValue;
	return batVoltage;
}

int8_t read_adc_all(uint8_t *adc_all_buff){
    /*0 to 7 PAC; 8 to 13 ADC*/
	int8_t pac_stat;
 	pac1952_busy();
	pac_stat = PAC194x5x_GetVIP_digital(pPACdevice,adc_all_buff,16);
	if(pac_stat != 0){
 		return 1; 
 	}	
 	get_uc_adc_all(&adc_all_buff[16]);
 	return 1;
}

int8_t get_sensor_values(uint8_t is_flush, uint8_t is_tx){
	static uint32_t sol_v_sum, load_i_sum, inv_v_sum, temp1_sum, vref_sum, temp2_sum;
	uint16_t _sol_v, _temp1, _temp2, _load_i, _inv_v, _v_ref;
	uint16_t moving_avg;
	uint8_t _adc_buff[16];
	int8_t pac_stat;
	uint32_t uint_pac_reg;
	int16_t int_pac_reg;
	float float_pac_val;
	if(test_error(ERR_NOT_CALIB)){
		memset((uint8_t *)&algo_param, 0, SUMMARY_LEN);
	}
	else{
		get_uc_adc_all(_adc_buff);
        _inv_v  = (_adc_buff[0] << 8) | _adc_buff[1];
		_sol_v  = (_adc_buff[2] << 8) | _adc_buff[3];
		_load_i = (_adc_buff[4] << 8) | _adc_buff[5];
		_temp1  = (_adc_buff[6] << 8) | _adc_buff[7];
        _v_ref  = (_adc_buff[8] << 8) | _adc_buff[9];
        _temp2  = (_adc_buff[10] << 8) | _adc_buff[11];
		if(is_flush){
            inv_v_sum  = (_inv_v << ADC_MUL_DIV_FACT);
			sol_v_sum  = (_sol_v << ADC_MUL_DIV_FACT);
			load_i_sum = (_load_i << ADC_MUL_DIV_FACT);
            temp1_sum  = (_temp1 << ADC_MUL_DIV_FACT);
            vref_sum   = (_v_ref << ADC_MUL_DIV_FACT);
            temp2_sum  = (_temp2 << ADC_MUL_DIV_FACT);
		}
		else{
            inv_v_sum += (_inv_v);
			sol_v_sum += (_sol_v );
			load_i_sum += (_load_i);
            temp1_sum += (_temp1);
            vref_sum += (_v_ref);
            temp2_sum += (_temp2);
		}
        moving_avg = (inv_v_sum >> ADC_MUL_DIV_FACT); //Moving average for inverter vol current
		inv_v_sum -= moving_avg;
		inverter_vol = ((float)moving_avg * uc_bat_v_const);
		
        moving_avg = (sol_v_sum >> ADC_MUL_DIV_FACT); //Moving average for solar voltage
		sol_v_sum -= moving_avg;
		algo_param.cur_sol_v = (float)moving_avg * uc_sol_v_const;

		moving_avg = (load_i_sum >> ADC_MUL_DIV_FACT); //Moving average for CT current
		load_i_sum -= moving_avg;
		algo_param.cur_ac_load_i = ((float)moving_avg / ct_load_i_const);

        moving_avg = (temp1_sum >> ADC_MUL_DIV_FACT); //Moving average for temperature
		temp1_sum -= moving_avg;
		temperature1 = (float)moving_avg * uc_temp1_const;//(float)((float)moving_avg * ((float)3.3/(float) 4096));

        moving_avg = (vref_sum >> ADC_MUL_DIV_FACT);    //Moving average for voltage reference
        vref_sum -= moving_avg;
        vol_ref = (float) ((float)moving_avg*((float)3.3/(float)4096));
        
        moving_avg = (temp2_sum >> ADC_MUL_DIV_FACT); //Moving average for temperature
		temp2_sum -= moving_avg;
		temperature2 = (float)moving_avg * uc_temp2_const;//(float)((float)moving_avg * ((float)3.3/(float) 4096));
        
        if(mains_sense()){
            mains=1;
        }
        else{
            mains=0;
        }     

		while(i2cFlag){};
		vTaskDelay(10/portTICK_RATE_MS);
		i2cFlag=1;   
	 	pac1952_busy();
		pac_stat = PAC194x5x_GetVIP_digital(pPACdevice, _adc_buff,16);
		if(pac_stat != 0){
	 		return 1; //1
			ESP_LOGI(TAG,"Error reading PAC1953 sensor");
	 	}
		i2cFlag=0;

		uint_pac_reg = (((uint16_t)_adc_buff[4] << 8) | _adc_buff[5]); // CH1 source
		algo_param.cur_bat_v = (float)uint_pac_reg * pac_bat_v_const;

	 	int_pac_reg = (((int16_t)_adc_buff[0] << 8) | _adc_buff[1]); // CH1 sense
	 	if(int_pac_reg & Bit(15)){ // convert 11 bit signed int to 16 bit signed int
            float_pac_val = ((float) int_pac_reg * pac_bat_dis_i_const) - pac_bat_dis_i_offset;
            algo_param.cur_chg_i = 0.0;
			algo_param.cur_dis_i = float_pac_val * -1;
            if(algo_param.cur_dis_i<0.0){
                algo_param.cur_chg_i = float_pac_val;
                algo_param.cur_dis_i = 0.0;
                pac_bat_dis_p = 0.0;
				pac_bat_chg_p = algo_param.cur_chg_i * algo_param.cur_bat_v;
            }
            else{
                pac_bat_dis_p = algo_param.cur_dis_i * algo_param.cur_bat_v;
                pac_bat_chg_p = 0.0;
            }
		}
        else{
            float_pac_val = ((float)int_pac_reg * pac_bat_chg_i_const) - pac_bat_chg_i_offset;
            algo_param.cur_chg_i = float_pac_val;
	 		algo_param.cur_dis_i = 0.0;
	 		pac_bat_dis_p = 0.0;
			pac_bat_chg_p = algo_param.cur_chg_i * algo_param.cur_bat_v;
        }

	 	int_pac_reg = (((int16_t)_adc_buff[2] << 8) | _adc_buff[3]); // CH2 sense
		algo_param.cur_sol_i = ((float)int_pac_reg * pac_sol_i_const * 1)-pac_sol_i_offset; 
		if(algo_param.cur_sol_i < 0){
			algo_param.cur_sol_i = 0.0;
			pac_sol_p = 0.0;
		}

		dc_load_i = ((algo_param.cur_sol_i + algo_param.cur_dis_i) - algo_param.cur_chg_i);
		if(dc_load_i < 0){
			dc_load_i = 0;
		}
		ac_load_p = algo_param.cur_ac_load_i * AC_POWER_CONST;	

		pac_sol_p = algo_param.cur_sol_i * algo_param.cur_sol_v;
		
		if(pac_sol_p < pac_bat_chg_p){
			mains_chg_p = (pac_bat_chg_p - pac_sol_p);
			sol_bat_p = pac_sol_p;
		}
		else{
			mains_chg_p = 0.0;
			sol_bat_p = pac_bat_chg_p;
		}

		float_pac_val = (algo_param.cur_dis_i * LVD_COEFF);
		bat_frc_trip_exit_v = (bat_frc_trip_exit_v_thr - float_pac_val);
		bat_mains_chg_in_v = (bat_mains_chg_in_v_thr - float_pac_val);
		sol_bat_chg_i_diff = (algo_param.cur_sol_i - algo_param.cur_chg_i);
	}
	return 1;
}