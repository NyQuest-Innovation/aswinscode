#ifndef _esp32s2_algorithm_h
#define _esp32s2_algorithm_h

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

#define SYS_TYPE_12V	1
#define SYS_TYPE_24V 	2
#define TUBULAR 		1
#define NON_TUBULAR 	0 
#define SUMMARY_LEN		84  // sizeof(algo_param)

typedef enum{
  lops_send_useable_soc_to_server,
  lpos_data_not_available,
  lpos_data_available,
  lpos_link_not_connected,
  lpos_link_connected,
  lpos_waiting_for_response,
  lpos_response_received,
  lpos_communication_failed,
  time_not_synced,
  waiting_for_time_synced,
}lpos_parameters_t;

typedef struct operations_parameters{
	uint8_t status;
	uint8_t wait_time;
}operations_parameters_t;

typedef struct  algo_param_tag{
	uint8_t date_time[6];
	float cur_bat_v;
	float cur_chg_i;
	float cur_dis_i;
	float cur_sol_v;
	float cur_sol_i;
	float cur_ac_load_i;
	uint16_t cur_temp;
	float bat_cur_cap_coul;
	uint8_t dev_algo_state;
	float sol_e;
	float sol_bat_e;
	float bat_dis_e;
	float bat_ac_chg_e;
	float ac_load_e;
	uint16_t num_sum_samples;
	uint8_t log_type;
	uint8_t state_change_reason;
	uint16_t dev_stat;
	uint16_t algo_stat;
	float cur_inv_v;
	uint8_t error;
	uint8_t daystartflag;
	uint8_t resetflag;
	float utility_savings;
}algo_param_t;

typedef struct log_soc_voltage
{
    float log_SOC[20];
    float log_voltage[20];
    float log_current[20];
    uint8_t date_time[5];
}load_soc_v;

typedef struct bat_equ_param_tag
{
	uint8_t 	equ_sol_sw_stat;
	uint8_t 	equ_mode;
	uint16_t 	equ_on_chk_time;
	uint16_t 	equ_on_time_counter;
}bat_equ_param_t;

typedef enum{
	ALGO_STATE_INV_5				= 0x05,
	ALGO_STATE_SOL_CHG_3			= 0x03,
	ALGO_STATE_BAT_FRC_TRIP_4		= 0x04,
	ALGO_STATE_BAT_SOL_FRC_TRIP_6	= 0x06,
	ALGO_STATE_SOL_INV_DIS_7		= 0x07,
	ALGO_STATE_CHARG_INHIBIT_1		= 0x01,
}algorithm_states_name;

extern struct tm* date_time;
extern algo_param_t algo_param;

extern float pac_bat_v_const;
extern float pac_sol_v_const, uc_bat_v_const, uc_sol_v_const,uc_temp1_const,uc_temp2_const;
extern float pac_bat_chg_i_const, pac_bat_dis_i_const, pac_bat_chg_i_offset, pac_bat_dis_i_offset, pac_sol_i_const,pac_sol_i_offset;
extern float pac_bat_p_const, pac_sol_p_const;
extern float bat_mains_chg_in_v, bat_mains_chg_in_v_thr, bat_mains_chg_out_v;
extern float dc_load_i,sol_bat_chg_i_diff;
extern float pac_sol_p, pac_bat_chg_p, pac_bat_dis_p, ac_load_p;
extern float bat_frc_trip_exit_v, bat_frc_trip_exit_v_thr;
extern float mains_chg_p, sol_bat_p;
extern uint16_t bat_max_volt_adc, num_days;
extern int16_t ct_load_i_offset;
extern float ct_load_i_const;
extern uint8_t debug_log_flag, one_hour_log_flag, force_trip_pause_flag;
extern uint8_t is_day_flag, solar_raised;
extern uint16_t force_trip_pause_count;

extern float useable_SOC1,eve_soc_corr,night_soc_corr;
extern uint8_t dev_state_flag;

extern load_soc_v  load;

#define LVD_COEFF		0.020 // 20 mV/A 
#define SOL_RECON_OFFSET	0.1 // 100mV

/*The minimum solar current has been changed from 1.0A to 0.1A*/
/*This is done due to the accuracy of PAC over ZXCT*/
/*The change effects from v7.0.12.2*/

#define MIN_SOL_I			1.0 // Amp
#define MIN_DC_LOAD_I		0.8 //2.0 // Amp
#define MIN_DIS_I			0.8 // Amp
#define MIN_CHG_I 			0.8 // Amp

#define SOL_BAT_CHG_I_DIFF_THR	2.0 // Amp

#define ALGO_STATE_INV_5				0x05
#define ALGO_STATE_SOL_CHG_3			0x03
#define ALGO_STATE_BAT_FRC_TRIP_4		0x04
#define ALGO_STATE_BAT_SOL_FRC_TRIP_6	0x06
#define ALGO_STATE_SOL_INV_DIS_7		0x07
#define ALGO_STATE_CHARG_INHIBIT_1		0x01

#define DEBUG_LOG_INTERVAL_DUR_SEC		30 // 30 sec
#define MIN_MAINS_TOPUP_DUR_SEC			1800 // 30 min

#define LOW_SOL_DETECT_DUR_SEC			180 // 3 min
#define SOL_V_CHECK_DUR_SEC				300 // 5 Min
#define LOW_DC_LOAD_DETECT_DUR_SEC		30 // 30 sec
#define DC_OVER_LOAD_DETECT_DUR_SEC		2    // 2 sec
#define FRC_TRIP_PAUSE_DUR_SEC			1800 // 30 min
#define NO_SOLAR_DETECT_DUR_SEC			86400 // 1 day

//#define IS_DAY_INIT_VAL		0xFF
#define NOW_DAY_TIME		0x01
#define NOW_NIGHT_TIME		0x00

#define SOL_LOW_H_THR 		10.0
#define SOL_LOW_L_THR       2.0
#define SOL_HIGH_THR        11.0

#define ALGO_TIME_MORN_PEAK_CHG_INH_START ((5 << 8) | 30)
#define ALGO_TIME_MORN_PEAK_CHG_INH_STOP  ((8 << 8) | 00)

#define ALGO_TIME_EVEN_PEAK_CHG_INH_START ((18 << 8) | 30)
#define ALGO_TIME_EVEN_PEAK_CHG_INH_STOP  ((22 << 8) | 30)

extern void turtle_algorithm();
extern void pause_force_trip();
extern void show_led();
extern void init_algo();
extern void clear_summary();
extern void calc_summary();
extern void load_sensor_constants();
extern void bat_equ_timer_stop();
extern void bat_equ_timer_start(uint16_t on_chk_timr_ms);
extern void halt_cpu();
extern void check_battery_voltage();
extern void Solar_Energy_Bucketing();
extern void Load_Energy_Bucketing();
extern void state_change_algorithm();
extern void Dynamic_FT_Exit_Algorithm();
extern uint32_t Dynamic_FT_Entry_Time_Calculate();
extern void Dynamic_FT_Entry_Algorithm();
extern void reset_parameters();
extern void reset_solar_power_bucket();
extern void reset_load_power_bucket();
extern void sun_down_routine();
extern void store_in_eeprom();
extern void mains_detection(void);
extern void sun_dail(void);
extern void collect_sensor_data(void);
extern void check_time(void);
extern uint8_t detect_solar_over_current(void);
extern void solar_over_current_protection(void);
extern void load_energy_per_hour_bucketing(void);
extern void solar_energy_per_hour_bucketing(void);
extern void received_solar_energy_calculation(void);
extern void sunup_routine(void);
extern void prepare_calibration_data(char *out);
extern void singlePressEffectOnAlgorithm(void);
extern void calculate_utility_savings(void);
#endif