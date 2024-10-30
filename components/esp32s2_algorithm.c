#include "esp32s2_algorithm.h"
#include "esp32s2_buff_operations.h"
#include "esp32s2_data_log.h"
#include "esp32s2_parse_data.h"
#include "esp32s2_pac1952.h"
#include "esp32s2_buzzer.h"
#define TAG "inside algorithm.c"

char dev_id[12];
/*Variable declarations*/
time_t timeinfo = 0;
struct tm* date_time;

algo_param_t algo_param;

volatile uint16_t system_flag;
volatile uint16_t common_flag;
volatile uint8_t server_flag;
volatile uint8_t error_flag;

//Variable initialization

float dc_load_i, sol_bat_chg_i_diff;

//Variables for calibration
float pac_bat_v_const, pac_sol_v_const, uc_bat_v_const, uc_sol_v_const,uc_temp1_const,uc_temp2_const;
float pac_bat_chg_i_const,pac_bat_dis_i_const,pac_bat_chg_i_offset,pac_bat_dis_i_offset, pac_sol_i_const,pac_sol_i_offset;
float pac_bat_p_const, pac_sol_p_const;
int16_t ct_load_i_offset;
float ct_load_i_const;

//Variables for summary
float pac_sol_p, pac_bat_chg_p, pac_bat_dis_p, ac_load_p;
float mains_chg_p, sol_bat_p;


//Variables associated with battery voltage
float bat_max_v, bat_frc_trip_exit_v, bat_frc_trip_exit_v_thr,useable_bat_frc_trip_exit_v, bat_sol_con_v;
uint16_t bat_max_volt_adc;
float bat_mains_chg_in_v, bat_mains_chg_in_v_thr, bat_mains_chg_out_v;

//Variables associated with load
float max_dc_load_i, max_ac_load_i;


uint8_t is_day_flag=0, bat_sol_equ_intr_cnt=0,eve_dy_exit=0,dy_exit=0;
uint16_t bat_sol_equ_dur_cnt=0;

uint8_t morn_chg_inh_flag=0, even_chg_inh_flag=0, bat_sol_equ_intr=0;
uint16_t bat_sol_equ_dur=0;

uint8_t algo_interval_count = 0, is_time_to_log,useable_soc_recal_cnt=0;

uint8_t debug_log_flag = 1, one_hour_log_flag, force_trip_pause_flag = 0;
uint16_t force_trip_pause_count = 0;
uint32_t no_sol_check_count = 0;
uint8_t power_trip_flag, is_time_to_equ=0, bat_full_flag;
uint8_t is_sol_sensed, no_inh_flag, peak_inh_flag, no_frc_trip_flag;

uint8_t low_sol_flag, dc_over_load_flag, dc_under_load_flag,need_solar_mains_top_up;
uint8_t ready_for_frc_trip, mains_chg_flag=0, mains_sol_chg_flag=0, pac_error_flag, pac_error_flag_cnt=0, exception_flag;

uint16_t low_sol_check_count, over_load_check_count, under_load_check_count, mains_topup_count;
uint16_t sol_v_chk_count, num_days;
uint8_t frc_trip_count,prev_frc_trip_count,prev_ft_count=0;
uint8_t prev_hour;
algo_param_t algo_param;
load_soc_v  load;
bat_equ_param_t bat_equ_param;
uint32_t total_time=0;

//v7 Variables 
//Newly included variables
uint8_t battery_low=0,mains_sense_dur=0;
uint8_t enable_cal_useable_soc=0,abs_index=0,battery_type=0,equalization_repeat_counter=0,is_time_to_cal_useable_soc=0,is_time_to_cal_useable_soc_entry=0,load_SOC_vol_index=0,useable_SOC_written=0,useable_soc_cal_failed=0,cal_useable_soc_thr2_cnt=0,cal_useable_soc_thr1_cnt=0;
uint16_t mains_fail_counter=0,equalization_failure_counter=0;
extern uint8_t device_state,turt_state;
float Rated_Bat_AH=0,Bat_SOC=0,battery_high_set=0,VFT_Exit_Higher=0,VFT_Exit_Lower=0;
uint8_t bat_abs_intr_cnt=0,bat_abs_intr=0;
float absorption_current[180];
float MA_absorption_current=0.0;
float previous_voltage=0.0,useable_soc_multiplier=0.0,useable_SOC1=0.0,useable_SOC2=0.0,Useable_SOC_Reducer=0.0;

uint8_t Solar_Time=0,Morning_Sunhour=6;
uint16_t solar_time_seconds=0;
float Today_Recieved_Solar_Energy=0.0,Morning_Solar_Bucket=0,solar_power_bucket[12],Afternoon_Solar_Bucket=0,Day_Morning_Solar_Bucket[3],Day_Afternoon_Solar_Bucket[3],MA_Morning_Solar_Bucket,MA_Afternoon_Solar_Bucket;
float Day_Load_Bucket,Night_Load_Bucket,load_power_bucket[24],Day_Morning_Load_Bucket[7],Day_Night_Load_Bucket[7];
uint8_t Sundown_Time=0,Previous_Day_Morning_Sunhour=6,Previous_Day_Sundown=12;

uint8_t Dynamic_FT_Exit_Enable,Correction_Factor_Enable,Dynamic_FT_Exit,Dynamic_FT_Entry_Enable;
float Correction_Factor;

float B1,B2,C2,AH_Efficiency,FT_Constant;
uint32_t FT_Entry_Time;
uint8_t Dynamic_FT_Enter,Dynamic_FT_Entered;

float SOC_Error=0.0,SOC_Err_Inc=0.0,MA_SOC_Err[4];
uint8_t SOC_Err_Index=0,need_eq_after_abs=0;

uint32_t estimated_ft_entry_time=0,check_ft_entry_time=0;
uint8_t is_time_to_dynamic_ft_entry=0;
float sol_mains_chg_cur_thr=25.0;
float max_solar_current=35.0;
uint8_t stop_solar_chg=0;
uint16_t stop_solar_chg_cnt=0;

float total_bat_capacity=0.0,total_bat_capacity_age_factor=0.4;
float Bat_DoD=0.0;

uint8_t dev_type;
float bat_vol,useable_SOC;
//Testing Purpose
extern uint8_t sol_data[3];
uint8_t user_enable=0;
uint8_t one_time=1;
uint16_t state_flag=0;
uint8_t bat_age=0;
float sol_inst;

uint8_t ft_entry_start_day=0,re_cal_useable_soc_count=0;
uint8_t ft_exit_start_day=0;
uint16_t abs_mains_fail_tolerance=0;
uint16_t equ_fail_tolerance=0;
float useable_soc_1_tolerance=0.0;
float useable_soc_2_tolerance=0.0;
float equ_hys=0.0;
uint8_t useable_soc_1_day_tolerance=0;
uint8_t useable_soc_2_day_tolerance=0;
uint8_t week_completed,weekly_count;
uint8_t soc_reset_flag=0;
float bat_voltage_settable;
uint8_t useable_soc_recal_dur;
uint8_t useable_soc_cal_cnt=0;
uint8_t max_frc_per_day=2;
uint8_t eve_soc_error=0;
uint8_t state_change_reason=0,prev_dev_state=0,one_sec_log_time_out=0,state_change_status=0,state_change_log_interval_duration_seconds=0;
uint8_t solar_cycling=0,dev_state_flag=0;
uint16_t cycling_solar_present_time,cycling_solar_absent_time,total_solar_cycling,solar_correction_cnt,solar_equ_cnt=0;
float cycling_energy=0.0,solar_cycling_energy=0.0;
extern uint8_t test_bit;
extern uint16_t server_stat_bit;
uint8_t sunup_count=0,sundown_count=0,sol_dis_count=0;
uint8_t mains_available=0,switch_pos=0,first_day_soc_reset_flag=0,button_press=0,dynamic_ft_complete=0;
uint8_t fail,sun_down_error=0;
uint32_t server_ft_exit_time=0;
extern float inverter_vol;
float weight1=0.0,weight2=0.0,eve_soc_corr,night_soc_corr;
uint8_t sss=0,sath=0,satm=0,sasd=0,sss_start=0,prev_sss=0,useabe_SoC_start_time=0;
uint16_t sssd=0,sss_start_dur=0;
uint32_t sss_time=0;

uint8_t sunup_conf_enable=0,readyforsunup=0;
uint16_t timetosunup=0;
extern uint8_t user_device_off;

uint8_t enable_early_ft_entry=0, early_ft_entry=0;

extern uint8_t stop_algorithm;

extern https_request_buffer_body_t https_request_buffer_body;

float utility_saving_array[4]={0.00139,0.0028,0.00835,0.0125};

uint8_t prevent_solar_charging=0, prevent_solar_charging_timeout=0; 

extern xSemaphoreHandle connectionSemaphore;
extern xSemaphoreHandle i2cSemaphore;

void clearOverVoltageFlag(){
    // xSemaphoreTake(i2cSemaphore,portMAX_DELAY);
    // getOverVoltageStatus();
    // xSemaphoreGive(i2cSemaphore);
}

static inline void
prevent_solar_charging_of_device(void) {
    prevent_solar_charging=1;
    prevent_solar_charging_timeout=0;
}

/*
 * params: none
 * return value: none
 * Brief: The following function gets the time from RTC
 *        It also parses the time value in D/M/Y/H/M/S format for use in data logging
 *        The time acquired is used only for data logging. Sundail is the major time source for algorithm.
 */
void check_time(void){
    ESP_LOGI(TAG,"Getting time");
	time(&timeinfo);
    date_time = localtime(&timeinfo);
    ESP_LOGI(TAG,"%02d-%02d-%02d %02d:%02d:%02d",date_time->tm_mday,1+(date_time->tm_mon),date_time->tm_year+1900,date_time->tm_hour,date_time->tm_min,date_time->tm_sec);
}

void state_change_algorithm() {   
    /*
     * Condition to check whether solar is available. 1) Checks if Sol_V>Bat_V 2)Checks the flag set for night indication.  
     * This routine is entered in the morning when solar comes in and the system is still in night mode
     */
  if(algo_param.cur_sol_v > SOL_HIGH_THR && (is_day_flag == NOW_NIGHT_TIME) && ((readyforsunup==0x01)||(sunup_conf_enable==0x00)))
	{
    /*
     * After detecting solar voltage, the algorithm waits for 5 seconds to ensure sun up.
     */
       if(sunup_count<4)
       {
       sunup_count++;
       }
       else
       {
       sunup_count=0;
    /*
     * Indicates solar panel as been connected to the device. If solar sense is 0 it means solar panel is not connected to the device
     * It is indicated using 1 sec yellow led blink
     */
       sun_down_error = 0;
       is_sol_sensed  = 1;
       clear_system_flag(SF_SOL_DIS); 
    /*
     * This variable holds the total solar wattage received for the day.
     * It is updated every second.
     */
       Today_Recieved_Solar_Energy=0;
    /*
     * These two variables are used to run a clock based on sun up and sun down. 
     * The variable Solar_Time is used to track hour and the variable solar_time_seconds is used to track seconds
     * The hour variable will reset everyday morning when the solar comes in for the first time.
     * The seconds variable increments every one second, and when it reaches 3600 it is reset after incrementing hour by 1. 
     * These variables are extensively used in solar bucketing, load bucketing, server set state, exiting FT at 4:30PM, evening battery charge back and dynamic entry
     */
       Solar_Time=0;
       solar_time_seconds=0;
    /*
     * Feature to enter force trip if battery is 90% full
     */
     enable_early_ft_entry = 1;
    /*
     * Dynamic entry depends on solar energy received for the past three days and load energy observed for the last seven days.
     * If the user has a low load consumption during morning hours, the device cycles the solar energy. This might reflect as low solar energy received.
     * The variable is used to track the time taken for equalization and enter into dynamic force trip if equalization happened for more than 1 hour.
     */
       solar_equ_cnt=0;
    
    /*
     * The purpose of dynamic force trip entry is to utilize solar energy to optimal level possible.
     * After a dynamic entry is completed, the battery should not be charged using mains. This variables blocks the charging of battery from mains after a dynamic entry.
     */
       dynamic_ft_complete=0;

    /*
     * The algorithm buckets load energy only after a proper sun up and sundown has passed.
     * The time of installation of the device is unknown. Upon starting the solar hour and solar seconds are zero. 
     * On day 2, the solar time is reset on sun up, and sun down time is noted at sundown.
     * Only after this correction, the algorithm buckets the load.
     */
       if((num_days>2)&&!(dev_state_flag&Bit(6)))
       {
         Load_Energy_Bucketing();
       }
    
    /*
     * After bucketing the load, the variables used for bucketing are reset.
     */
       reset_load_power_bucket();
       reset_solar_power_bucket();
       
    /*
     * This variable is used to track the number of days passed after installation. The variable will get reset once battery is removed/after a reset either from WDT or server.
     */
       num_days++;
      
    /*
     * Enable sunup confidence check
     */
       if(num_days>2)
       {
           sunup_conf_enable = 1;
       }
       else
       {
           sunup_conf_enable = 0;
       }
       
       readyforsunup = 0;
    /*
     * On first day of installation the device doesn't force trip. It is done by setting the force trip count allowed for the day to maximum value.
     * During regular days the force trip count is set to zero in the morning when solar entry happens 
     */
       if(((num_days<=1)&&(!(dev_state_flag&Bit(2))))||(test_system_flag(SF_INV_FAIL)))
       {
            frc_trip_count = max_frc_per_day;
       }
       else
       {
            frc_trip_count = 0;  
            prev_frc_trip_count = 0;
       }

    /*
     * Dynamic force trip exit feature is enabled only on the 5th day after installation of device.
     * If a useable SoC calculation failure occurs, the feature gets delayed by a day.
     * This feature can be blocked from the server if required.
     */

       if((num_days>=ft_exit_start_day)&&!(dev_state_flag&Bit(4)))
       {
            Dynamic_FT_Exit_Enable=1;
       }

    /*
     * Dynamic force trip entry feature is enabled only on the 9th day after installation of device.
     * If a useable SoC calculation failure occurs, the feature gets delayed by a day.
     * This feature can be blocked from the server if required.
     */

       if((num_days>=ft_entry_start_day)&&!(dev_state_flag&Bit(3)))
       {
            Dynamic_FT_Entry_Enable=1;
            dynamic_ft_complete=0;
            is_time_to_dynamic_ft_entry=0;
       }
      
    /* 
     * Reset Dynamic Exit and Entry Flag
     */
       Dynamic_FT_Exit=0;
       Dynamic_FT_Enter=0;
       dy_exit=0;

    /* 
     * Battery absorption complete check. 
     */
        
    /*
     * Case 1: If there happens a mains fail on the previous night during absorption, SF_ABS_MF  flag is set.
     * If SF_ABS_MF flag is set, the absorption current is checked.
     */    
     if(test_system_flag(SF_ABS_MF)) 
     {
        for(int i=0;i<180;i++)
        {
        MA_absorption_current+=absorption_current[i];
        }           
        
        MA_absorption_current=MA_absorption_current/180;

    /*
     *  Case 1a: If the SF_ABS_MF flag is set the battery charging tail current is calculated. If it is less than 3A the absorption failed flag is set.Absorption current is set as 0.02 of rated battery AH.
     *  Absorption current value is a moving average value of charging current over a period of 3 minutes.
     */   
        if(MA_absorption_current<(Rated_Bat_AH/50.0))                                                                                                             
        {
        post_system_flag(SF_ABS_COMPLETED);
        clear_system_flag(SF_ABS_MF);
        clear_system_flag(SF_ABS_NEEDED);
        if(enable_cal_useable_soc)
        {
        soc_reset_flag=0;
        }
        }
    /*
     *  Case 1b: Else the SF_ABS_MF flag is reset and SF_ABS_COMPLETED flag is set to indicate the completion of absorption.
     */    
        else
        {
        post_system_flag(SF_ABS_NEEDED);
        clear_system_flag(SF_ABS_MF);
        clear_system_flag(SF_ABS_COMPLETED);
    /*
     * If the absorption was initiated for useable SoC calculation and if the absorption has failed, then useable SoC calculation is re-attempted the next day.
     * If already re-attempted, and it has failed again, the useable SoC calculation is considered a failure.
     */
        if(enable_cal_useable_soc)
        {
        if(re_cal_useable_soc_count>1)
        {
        enable_cal_useable_soc=0;
        need_eq_after_abs=0;
        post_system_flag(SF_USEABLE_SOC_FAILED);
        frc_trip_count = 0;  
        re_cal_useable_soc_count=0;
        //useable SoC Calculation failed
        }
        else
        {
        ft_exit_start_day++;
        ft_entry_start_day++;
        post_system_flag(SF_USEABLE_SOC_CAL);
        }
        }
        
        }
    }
    /*
     *  Case 2: If the flag is not set SF_ABS_COMPLETED flag is set to indicate completion of absorption.
     */    
    else
    {
        if(test_system_flag(SF_ABS_NEEDED)==0)
        {
        if(enable_cal_useable_soc)
        {
        soc_reset_flag=0;
        }
        post_system_flag(SF_ABS_COMPLETED);
        clear_system_flag(SF_ABS_MF);
        clear_system_flag(SF_ABS_NEEDED);
        }
    }
            
    /*
     *  This variable is used to count the availability of solar during the states 3,6 and 7 by checking the current.
     *  If the solar current is less than specified value for more than 5 minutes the device .
     */
        no_sol_check_count = 0;
    
    /*
     * This flag indicates day or night. It is set during sun up and reset during sundown.
     * This variable is sent to the server during payload. The sun up time of the device is detected by the server using this flag.
     * Using this time the server corrects for the state change entry time. This correction is considered because of IST issues.
     */
        is_day_flag = NOW_DAY_TIME;
        
    /*
     * This flag indicates whether to inhibit the battery or to continue with mains charging
     * This variable is used to enter into battery absorption
     */        
        no_inh_flag = 0;       

    /* Equalization is done only if the battery is a tubular battery.
     * Night before equalization absorption is done.
     * Equalization is done only if absorption is complete.
     * Periodic equalization interval can be set between 30 to 120days.
     */
       if(battery_type==TUBULAR)
       {
       bat_sol_equ_intr_cnt+=1;

       /*
        * It tracks the equalization day and initiates absorption the day before equalization.
        */
       if(bat_sol_equ_intr_cnt==bat_sol_equ_intr)
       {
       post_system_flag(SF_ABS_NEEDED);
       }
       
       /* 
        * This code initiates equalization.
        */
        if((((bat_sol_equ_intr != 0) && (bat_sol_equ_dur != 0 )) && (bat_sol_equ_intr_cnt > bat_sol_equ_intr)) || test_system_flag(SF_EQ_NEEDED)){
            if(test_system_flag(SF_EQ_NEEDED)){
                clear_system_flag(SF_EQ_NEEDED);
                equalization_repeat_counter++;
            /*
            * If equalization repeat counter reaches 2 set equalization failed flag and clear equalization needed flag.
            */
                if(equalization_repeat_counter>1)
                {
                clear_system_flag(SF_EQ_NEEDED);
                post_system_flag(SF_EQ_FAILED);
                is_time_to_equ=0;
                }
                else
                {
                is_time_to_equ=1;
                }
            }
            else{
                /*
                * Incase absorption initiated on the previous day is not complete, equalization is not performed.
                */ 
                if(test_system_flag(SF_ABS_NEEDED)){
                /*Postpone the equalization*/
                    is_time_to_equ = 0;
                    post_system_flag(SF_EQ_NEEDED);
                }
                else
                {
                is_time_to_equ = 1;
                }
            }
        }

                    
        /*
         * Case 2: During equalization day the battery cycles between battery voltage+(battery voltage)/4 and (battery voltage+(battery voltage)/4)-0.4
         */
        if((test_system_flag(SF_ABS_COMPLETED))&&(is_time_to_equ==1))
        {
            is_time_to_equ=1;
            bat_voltage_settable=bat_vol+(bat_vol/4.0);
            bat_max_volt_adc= (uint16_t) (bat_voltage_settable/uc_bat_v_const);
        }
        /*
         * Case 1: During regular days once battery voltage hits maximum and FT is not allowed to happen the battery cycles between bat_max_v and bat_max_v-0.4
         */
        else
        {
            is_time_to_equ=0;
            bat_voltage_settable=bat_max_v;
            bat_max_volt_adc= (uint16_t) (bat_voltage_settable/uc_bat_v_const);
        }
        }
        /*
         * If the battery is not of tubular type, then the battery is only cycled using solar.
         * Equalization is not performed on non-tubular batteries.
         */
        else
        {
            is_time_to_equ=0;
            bat_voltage_settable=bat_max_v;
            bat_max_volt_adc= (uint16_t) (bat_voltage_settable/uc_bat_v_const);
        }

        /*The absorption feature is available for all battery types*/
        /*Absorption can be done every 4 to 7days*/
        if(++bat_abs_intr_cnt>bat_abs_intr)
        {
            bat_abs_intr_cnt=0;
            post_system_flag(SF_ABS_NEEDED);
        }

        if(enable_cal_useable_soc==1)
        {
          frc_trip_count = max_frc_per_day;  
        }
            
        }
    }

    /*
     * This code detects whether the solar panel is disconnected or a sundown occurs.
     * During a solar panel disconnect, the solar voltage falls to zero in no time, where as during a sundown the voltage takes minutes to fall to zero.
     * This logic is used in determining the condition that occurred.
     */
    else if((algo_param.cur_sol_v < SOL_LOW_H_THR) && (algo_param.cur_sol_v > SOL_LOW_L_THR) && (is_day_flag == NOW_DAY_TIME))
	{
    
    /*
     * If the voltage goes below a certain specified value which is provided to detect sundown, the algorithm waits for 30 seconds to ensure it.
     */
        if(sundown_count<30)
        {
            sundown_count++;
        }
        else
        {
    /*
     * After two days from installation, after the voltage falls, check if the solar hour is greater than the previous day's sundown hour.
     * If the solar hour is greater, then it is considered as a sundown.
     */
            if(num_days>2)
            {
                if(Solar_Time>=(Previous_Day_Sundown-1))
                {
                    sun_down_routine();
                }
                else
                {
                sun_down_error=1;
                sol_dis_count=0;
                is_sol_sensed=0;
                reset_parameters();
                sundown_count=0;
                }
            }
            else
            {
                sun_down_routine();
                sundown_count=0;
            }
        }
    }
    
    /*
     * If the solar voltage drops less than a set voltage, and if it stays there for 20seconds it is considered as solar disconnect.
     */
    if((algo_param.cur_sol_v < SOL_LOW_L_THR) && (is_day_flag == NOW_DAY_TIME))
    {
        if(sol_dis_count<20)
        {
            sol_dis_count++;
        }
        else
        {
            sun_down_error=1;
            sundown_count=0;
            sol_dis_count=0;
            is_sol_sensed=0;
            reset_parameters();
            post_system_flag(SF_SOL_DIS);
        }
    }
    
    total_time=(((uint32_t)Solar_Time*(uint32_t)3600)+(uint32_t)solar_time_seconds);
    
    if(enable_early_ft_entry){
        if((total_time>7200)&&(algo_param.dev_algo_state!=ALGO_STATE_BAT_SOL_FRC_TRIP_6)&&(algo_param.dev_algo_state!=ALGO_STATE_BAT_FRC_TRIP_4)&&(Bat_SOC>0.9*total_bat_capacity)){
            early_ft_entry=1;
            enable_early_ft_entry=0;
        }
    }
    /*
     * Once dynamic exit is enabled, during state 4 and state 6 a rule check is done so that the FT exit happens.
     * This is to ensure that the battery is completely full at sundown.
     */
    if((algo_param.dev_algo_state == ALGO_STATE_BAT_SOL_FRC_TRIP_6) || (algo_param.dev_algo_state == ALGO_STATE_BAT_FRC_TRIP_4))
    {
        Dynamic_FT_Exit_Algorithm();
    }
    
    /* Mains Fail Check during Absorption.
     * On absorption days, mains fail during night time is detected.
     * If the mains fail persisted for more than three hours SF_ABS_MF is set. 
     * The duration of mains fail can be changed as required either from server or using APP.
     */
    if(no_inh_flag&&(is_day_flag==NOW_NIGHT_TIME))
    {
        if((algo_param.cur_dis_i>MIN_DC_LOAD_I)&&(test_system_flag(SF_ABS_MF)==0))
        {
            ++mains_fail_counter;
            if(mains_fail_counter>abs_mains_fail_tolerance)
            {
               post_system_flag(SF_ABS_MF); 
            }
        }
    }

    /* 
     * Battery SoC is reset after every absorption.
     * After absorption the moment the battery reaches maximum voltage, the soc is reset to full capacity
     */
    if((algo_param.cur_bat_v>bat_max_v)&&(soc_reset_flag==0))
    {
        soc_reset_flag=1;
        Bat_SOC=total_bat_capacity;
    /*
     * If the battery is of non-tubular type, after absorption complete the device is set to ready for useable SoC calculation.
     * This is executed, if the absorption was initiated because of a useable SoC calculation.
     */
    if(test_system_flag(SF_ABS_COMPLETED) && (battery_type!=TUBULAR))
    {
        if(enable_cal_useable_soc)
        {
           enable_cal_useable_soc=0;
           is_time_to_cal_useable_soc_entry=1;
           previous_voltage=bat_vol*1.034;
           frc_trip_count = max_frc_per_day;
           Dynamic_FT_Exit_Enable=0; //Dynamic FT does not happen during useable SoC calculation
        }
    }
    }       
    
    /*
     * On the first day of installation, the battery SoC is set to zero.
     * To correct it on the first day, when the battery voltage hits maximum for the first time, SoC is reset to full capacity.
     */
    if((algo_param.cur_bat_v>bat_max_v)&&(num_days==1))
    {
        Bat_SOC=total_bat_capacity;
    }   
    
    /* 
     * If the battery is tubular and if it is day time and equalization is being done, a counter runs for 6hours.
     * After 6hours if the battery soc has fallen below 95% the equalization process is considered as success.
     * 95% tolerance is given to accommodate any short power failure during equalization
     * Less than 95%, it is considered a failure
     */
    if(is_time_to_equ&&(is_day_flag==NOW_DAY_TIME)&&(battery_type==TUBULAR)) 
	{
            if(++bat_sol_equ_dur_cnt >= bat_sol_equ_dur)
            {
                is_time_to_equ=0;
                force_trip_pause_flag=0;
                force_trip_pause_count=0;
                bat_sol_equ_dur_cnt = 0;
               
                /* Success*/
                if((soc_reset_flag==1)&&(Bat_SOC>(0.95*total_bat_capacity)))
                {
                if(need_eq_after_abs==0)
                {
                    bat_sol_equ_intr_cnt = 0;
                }
                previous_voltage=bat_vol*1.034;
                need_eq_after_abs=0;
                if(enable_cal_useable_soc)
                {
                    enable_cal_useable_soc=0;
                    is_time_to_cal_useable_soc_entry=1;
                    frc_trip_count = max_frc_per_day;
                    Dynamic_FT_Exit_Enable=0; /*Dynamic FT does not happen during useable SoC calculation*/
                }
                post_system_flag(SF_EQ_COMPLETED);
                }
                /*Failure*/
                else
                {
                if(enable_cal_useable_soc)
                {
                /*
                 * If the equalization was initiated for useable SoC calculation and if the equalization has failed, then useable SoC calculation is re-attempted the next day.
                 * If already re-attempted, and it has failed again, the useable SoC calculation is considered a failure.
                 */
                fail=3;
                re_cal_useable_soc_count++;
                if(re_cal_useable_soc_count>1)
                {
                    enable_cal_useable_soc=0;
                    post_system_flag(SF_USEABLE_SOC_FAILED);
                    need_eq_after_abs=0;
                    re_cal_useable_soc_count=0;
                    //useable SoC Calculation failed
                }
                else
                {
                    ft_exit_start_day++;
                    ft_entry_start_day++;
                    post_system_flag(SF_USEABLE_SOC_CAL);
                }
                }
                else
                {
                    post_system_flag(SF_EQ_FAILED);
                }
                }
            }
	}
    
    /*
     * After the completion of absorption, (and equalization if tubular battery) the device is ready to force trip if a useable SoC calculation is needed.
     * The force trip is initiated if a mains fail occurs or if the solar time reaches specified time. 
     * This time can be changed using APP/Server
     */
    if(is_time_to_cal_useable_soc_entry==1)
    {
    if(mains_available==0)
    {
    force_trip_pause_flag=0;
    frc_trip_count=0;
    no_frc_trip_flag = 0;
    ready_for_frc_trip = 1;
    is_time_to_cal_useable_soc_entry=0;
    is_time_to_cal_useable_soc=1;
    }
    else
    {
    if(Solar_Time>=useabe_SoC_start_time)
    {
    force_trip_pause_flag=0;
    frc_trip_count=0;
    no_frc_trip_flag = 0;
    ready_for_frc_trip = 1;
    is_time_to_cal_useable_soc_entry=0;
    is_time_to_cal_useable_soc=1;
    }
    }
    }

    /*
     * To understand the state transition cause.
     * Every sec the variable is reset, a value will be written only during state transition.
     */
     state_change_reason=0;
    
    /*
     * If force trip count for the day reaches maximum allowable value, force trip will not happen for the reset of the day.
     */
    if((frc_trip_count >= max_frc_per_day))
	{
		no_frc_trip_flag = 1;
	}

    /*
     * A time period of 30 minutes is assigned between two consecutive force trips
     */
	if(force_trip_pause_flag)
	{
		if(force_trip_pause_count < FRC_TRIP_PAUSE_DUR_SEC)
		{
			force_trip_pause_count++;
		}
		else
		{
				force_trip_pause_count = 0;
				force_trip_pause_flag = 0;
		}

	}
    
    /*
     * At sundown if the battery soc is less than 0.9 of total capacity the battery is charged back to full capacity using mains
     */
    if(eve_soc_error)
    {
        if(Bat_SOC>=eve_soc_corr*total_bat_capacity)
        {
            eve_soc_error=0;
            clear_system_flag(EVE_SOC_ERR);
            mains_sol_chg_flag = 0;
            if(test_system_flag(SF_SMC))
            {
                clear_system_flag(SF_SMC);
            }
            mains_chg_flag=0;
        }
    }
    
    /*
     * If the battery voltages goes critical, the battery is charged using mains (and solar if available).
     */
	if((algo_param.cur_bat_v < bat_mains_chg_in_v) && (need_solar_mains_top_up == 0))
	{
		need_solar_mains_top_up=1;
        mains_sol_chg_flag = 1;
        post_system_flag(SF_SMC);
		mains_topup_count = 0;
	}

    /*
     * If mains+solar charging is allowed and if battery voltage reaches the required value the mains recharge is cut.
     * Since voltage of old batteries reach full voltage faster a time out of 30 minutes is provided. 
     * During this 30 minutes, once the battery voltage hits full the battery will be cycled using solar.
     */
	if(need_solar_mains_top_up)
	{
    if(mains_topup_count < MIN_MAINS_TOPUP_DUR_SEC)
    {
        mains_topup_count++;
    }
    if((Bat_SOC >= night_soc_corr*total_bat_capacity) && \
        (mains_topup_count >= MIN_MAINS_TOPUP_DUR_SEC))
    {
        clear_system_flag(NGT_SOC_ERR);
        need_solar_mains_top_up = 0;
        mains_sol_chg_flag = 0;
        if(test_system_flag(SF_SMC))
        {
          clear_system_flag(SF_SMC);
        }
        mains_chg_flag=0;
        mains_topup_count = 0;
    }
	}

    /*
     * If solar panel is disconnected from device for some reason.
     * If both solar voltage and solar current are less than minimum for 1day.
     * solar sensed is reset and LED yellow blinks every one second.
     */
     
	if((algo_param.cur_sol_v < bat_max_v) && (algo_param.cur_sol_i < MIN_SOL_I))
	{
    if(no_sol_check_count < NO_SOLAR_DETECT_DUR_SEC)
    {
        no_sol_check_count++;
    }
    else
    {
        is_sol_sensed = 0;
    }
	}
	else
	{
		no_sol_check_count = 0;
	}

    /*
     * During state 3, state 6 and state 7 since solar switch is closed, the voltage across solar panel will be equal to battery voltage.
     * Hence to check for presence of solar the solar current is checked.
     * If the current is less than the minimum solar current for 3 minutes low solar flag is set.
     * This flag is used to exit from states in which the solar switch is closed.
     */
    
    
	if((algo_param.dev_algo_state == ALGO_STATE_BAT_SOL_FRC_TRIP_6) || \
		(algo_param.dev_algo_state == ALGO_STATE_SOL_INV_DIS_7)||(algo_param.dev_algo_state == ALGO_STATE_SOL_CHG_3))
	{
		if(algo_param.cur_sol_i < MIN_SOL_I)
		{
			if(low_sol_check_count < LOW_SOL_DETECT_DUR_SEC)
			{
				low_sol_check_count++;
			}
			else
			{
				low_sol_flag = 1;
				low_sol_check_count = 0;
			}
		}
		else
		{
			low_sol_flag = 0;
			low_sol_check_count = 0;
		}
	}
	else
	{
		low_sol_flag = 0;
		low_sol_check_count = 0;
	}	


    /*
     * During state 4 and state 6 to prevent overload trip of inverter, the current is being checked.
     * If current is more than the maximum allowable current for more than 5sec the overload flag is set.
     */
	if((algo_param.dev_algo_state == ALGO_STATE_BAT_SOL_FRC_TRIP_6) || \
		(algo_param.dev_algo_state == ALGO_STATE_BAT_FRC_TRIP_4))
	{
		if(dc_load_i > max_dc_load_i)
		{
            is_time_to_log=1;
			if(over_load_check_count < DC_OVER_LOAD_DETECT_DUR_SEC)
			{
				over_load_check_count++;
			}
			else
			{
				dc_over_load_flag = 1;
				over_load_check_count = 0;
			}
		}
		else
		{
			dc_over_load_flag = 0;
			over_load_check_count = 0;
		}
     /*
      * The following code checks for the failure of the inverter.
      * An inverter connected to battery will consume a no-load current of 1.5A minimum.
      * If the battery discharge current during FT is less than this minimum value, then it means the inverter has failed.
      */
#warning "Inverter fail detection enabled"
#if 1

		if(dc_load_i < MIN_DC_LOAD_I)
		{
            is_time_to_log=1;
			if(under_load_check_count < LOW_DC_LOAD_DETECT_DUR_SEC)
			{
				under_load_check_count++;
			}
			else
			{
				dc_under_load_flag = 1;
				under_load_check_count = 0;
			}
		}
		else
		{
			dc_under_load_flag = 0;
			under_load_check_count = 0;
		}
#endif
	}
	else
	{
		dc_over_load_flag = 0;
		dc_under_load_flag = 0;
		over_load_check_count = 0;
		under_load_check_count = 0;
	}

    /*
     * During equalization and force trip pause for 30mins and no force trip for the day.
     * Ready for force trip flag is cleared.
     * Once it is cleared the algorithm can go to any state except 4 and 6
     */
     
	if((is_time_to_equ == 0) && (force_trip_pause_flag == 0) && \
		(no_frc_trip_flag == 0)&&(test_system_flag(SF_INV_FAIL)==0))
	{
		ready_for_frc_trip = 1;
	}
	else
	{
		ready_for_frc_trip = 0;
	}

     
    if(test_system_flag(SF_INV_FAIL))
    {
        if(dc_load_i > MIN_DC_LOAD_I)
        {
            is_time_to_log=1;
            clear_system_flag(SF_INV_FAIL);
            frc_trip_count = prev_frc_trip_count;
        }
    }
         
    /*
     * If peak inhibition flag is set mains charging is denied
     */
	if(peak_inh_flag)
	{
		mains_chg_flag = 0;
	}
	else
	{
    /*
     * During absorption or battery critical, battery is charged from mains by setting mains_chg_flag
     */
		if((no_inh_flag == 1) || (need_solar_mains_top_up==1))
		{
               mains_chg_flag = 1;
		}
		else
		{
			mains_chg_flag = 0;
		}
	}


    /*
     * Dynamic force trip exit feature is provided to ensure that the battery is fully charged at sundown.
     * Forecasting the previous solar pattern and the present battery SoC the decision to exit FT is done.
     */
    if(((algo_param.dev_algo_state == ALGO_STATE_BAT_SOL_FRC_TRIP_6) || \
		(algo_param.dev_algo_state == ALGO_STATE_BAT_FRC_TRIP_4))&&(is_time_to_cal_useable_soc==0)&&(is_time_to_dynamic_ft_entry==0)&&(is_day_flag == NOW_DAY_TIME)&&(sss_start==0))
    {
        if(total_time>(((uint32_t) Previous_Day_Sundown *(uint32_t) 3600)-(uint32_t)5400))
        {
            Dynamic_FT_Exit_Enable=0;
            Dynamic_FT_Exit=0x00;
            no_frc_trip_flag = 1;
            state_change_reason=25; //Evening FT Exit
            eve_dy_exit=1;
            ready_for_frc_trip=0;  
            frc_trip_count=max_frc_per_day;
            
            if(is_time_to_dynamic_ft_entry)
            {
                if(is_day_flag==NOW_NIGHT_TIME)
                {
                    dynamic_ft_complete=1;
                }
                is_time_to_dynamic_ft_entry=0;
            }
        }
    }
    
    /*
     * If useable soc calculate start is enabled the device moves to state 4. 
     */
	if(is_time_to_cal_useable_soc)
    {
        if(algo_param.dev_algo_state!=ALGO_STATE_BAT_FRC_TRIP_4)
        {
           algo_param.dev_algo_state=ALGO_STATE_BAT_FRC_TRIP_4; 
           state_change_reason=24; //Useable SoC FT entry
        }
    }
    
    /*
     * If dynamic force trip entry enable feature is enabled.
     * The system time is checked with the estimated ft entry time calculated and sundown.
     * Required actions are taken based on the rule check.
     */
       Dynamic_FT_Entry_Algorithm();
    
    /*
     * At sundown if the battery SoC falls below 90% of total capacity charge back to 90%.
     * The battery will not be charged back if the battery was discharged during dynamic entry. 
     */
    if((is_day_flag == NOW_NIGHT_TIME)&&(Bat_SOC<(night_soc_corr*total_bat_capacity))&&(is_time_to_dynamic_ft_entry==0)&&(no_inh_flag==0)&&(num_days!=0)&&(dynamic_ft_complete==0)&&(is_time_to_cal_useable_soc==0)&&(is_sol_sensed!=0))
    {
        need_solar_mains_top_up=1;
        post_system_flag(NGT_SOC_ERR);
    }
    
    
    /*
     * This variable indicates the state of the algorithm. 
     * This variable is sent to the server and is mainly used for debugging purpose
     */
       
    state_flag=(uint16_t) (sun_down_error<<14)|(is_day_flag<<13)|(mains_available<<12)|(is_sol_sensed<<11)|(is_time_to_dynamic_ft_entry<<10)|(enable_cal_useable_soc<<9)|(mains_chg_flag<<8)|(need_solar_mains_top_up<<7)|(Dynamic_FT_Entry_Enable<<6)|(Dynamic_FT_Exit_Enable<<5)|(is_time_to_cal_useable_soc_entry<<4)|(is_time_to_cal_useable_soc<<3)|(no_inh_flag<<2)|(is_time_to_equ<<1)|(need_eq_after_abs);
    
    /*
     * If the rule check of dynamic exit is positive, exit flag is set.
     * Ready for force trip flag is reset and no more force trip can happen for the day.
     */
    if(Dynamic_FT_Exit==0x01) //Solar Bucketing Code
    {
        
        if(is_time_to_dynamic_ft_entry)
        {
            if(is_day_flag==NOW_NIGHT_TIME)
            {
                dynamic_ft_complete=1;
            }
            is_time_to_dynamic_ft_entry=0;
        }
        
        if(test_common_flag(CF_FT_EXIT))
        {
            state_change_reason=27; //Server set dynamic exit
            clear_common_flag(CF_FT_EXIT);
            i2c_eeprom_write_buff(0xFE7F,(uint8_t *) &common_flag,2);
        }
        else
        {
            state_change_reason=23; //Dynamic FT exit
        }
        /*This is to avoid race condition in FT Entry and Exit*/
        Dynamic_FT_Exit_Enable=0;
        Dynamic_FT_Exit=0x00;
        no_frc_trip_flag = 1;
        force_trip_pause_flag = 1;
        force_trip_pause_count = 0;
        dy_exit = 1;
        ready_for_frc_trip=0;
    }
    
    /*
     * If solar panel is connected and there is no error in pac current sensing, the algorithm is enabled
     */
    if(is_sol_sensed && (pac_error_flag == 0) && (pac_error_flag_cnt==0))
	{
        // WDT_Fcn(); 
		switch(algo_param.dev_algo_state)
		{
            /*
             * State five the load and battery are on mains.
             * If the mains fail, then it is called inverter mode
             */
			case ALGO_STATE_INV_5:
                clearOverVoltageFlag();
				bat_full_flag = 0;
				if(exception_flag == 0)
				{
                    /*If there is mains and if mains recharge is not required the device moves to state 1*/
                    /*Solar and mains charge the battery together*/
                    if(sss_start)
                    {
                        //Do Nothing   
                    }
					else if((algo_param.cur_sol_v > bat_max_v)&&(algo_param.cur_bat_v<bat_sol_con_v)&&((mains_sol_chg_flag==1)||(mains_chg_flag==1)))
                    {
                        if(mains_sol_chg_flag==1)
                        {
                            mains_sol_chg_flag = 1;
                            state_change_reason=1; //Mains+solar charging
                        }
                        else
                        {
                            state_change_reason=4; //Mains charging necessary and mains is available
                        }

                        if (prevent_solar_charging==0){
                            algo_param.dev_algo_state = ALGO_STATE_SOL_INV_DIS_7;
                        }
                    }
                    else if((algo_param.cur_dis_i < MIN_DIS_I) && \
						(mains_chg_flag == 0)&&(mains_sol_chg_flag == 0))
					{
						algo_param.dev_algo_state = ALGO_STATE_CHARG_INHIBIT_1;
                        solar_cycling=0;
                        state_change_reason=2; //Mains available
					}
                    /*If mains fail, solar is available and battery is not full the algorithm goes to state 7*/
					else if(((algo_param.cur_dis_i > MIN_DIS_I) || (algo_param.cur_chg_i < MIN_CHG_I)) && \
							(algo_param.cur_bat_v < bat_sol_con_v) && \
							(algo_param.cur_sol_v > bat_max_v)&&(mains_chg_flag==0)&&(mains_sol_chg_flag == 0))
					{
                        if(prevent_solar_charging==0){
                        state_change_reason=3;

						algo_param.dev_algo_state = ALGO_STATE_SOL_CHG_3; //Mains failed and solar available
                        solar_cycling=1;
                        }
					}
				}	
				break;
            /*State 1 is the state where the battery is neither charged from solar nor from mains*/
            /*This state occurs during non-absorption days during night, during equalization and during peak inhibition if configured*/
            /*This is a major state. It can branch to any state except state 6*/
                
			case ALGO_STATE_CHARG_INHIBIT_1:
                clearOverVoltageFlag();
				if(exception_flag == 0)
				{
                    /*In order to prevent the sudden inrush of current during state 1 to state 5 transition, it has to go through state 4*/
                    if(sss_start)
                    {
                        
                    }
					else if(((mains_chg_flag == 1)||(mains_sol_chg_flag==1)))
					{
						switch_state_4_5();
                        state_change_reason=5; //Mains charging necessary
                        solar_cycling=0;
					}
                    /*If night time and dynamic ft entry is set it moves to state 4*/
                    /*It is made sure that ac load is less than the maximum load current the inverter can deliver*/
                    else if((is_day_flag==NOW_NIGHT_TIME)&&(is_time_to_dynamic_ft_entry)&&(algo_param.cur_ac_load_i < max_ac_load_i))
                    {
                        algo_param.dev_algo_state = ALGO_STATE_BAT_FRC_TRIP_4;
                        state_change_reason=6; // Dynamic FT Entry
                        solar_cycling=0;
                    }
                    /*If battery is full, solar voltage is greater than max battery voltage, ac load is less than overload value, ft is allowed*/
					else if((bat_full_flag == 1) && \
							((algo_param.cur_sol_v > bat_max_v)||(early_ft_entry)) && \
							(algo_param.cur_ac_load_i < max_ac_load_i) && \
							(ready_for_frc_trip == 1))
					{
                        if(early_ft_entry){
                            early_ft_entry=0;
                            state_change_reason=36; //Early FT entry
                        }
                        else{
                            state_change_reason=7; //Regular FT entry
                        }
                        enable_early_ft_entry=0;
                        if(num_days==1)
                        {
                            Bat_SOC=total_bat_capacity;
                        }
						algo_param.dev_algo_state = ALGO_STATE_BAT_FRC_TRIP_4;
                        
                        solar_cycling=0;
					}
                    /*If mains failed*/
					else if(dc_load_i > MIN_DC_LOAD_I)
					{
                        /*If solar is available move to state 3*/
						if((algo_param.cur_sol_v > bat_max_v))
						{
                            if (prevent_solar_charging==0){                     
							    algo_param.dev_algo_state = ALGO_STATE_SOL_CHG_3;
                                state_change_reason=3; //Mains failed and solar is available
                                solar_cycling=1;
                            }
						}
                        /*If solar is unavailable move to state 5*/
						else
						{
							algo_param.dev_algo_state = ALGO_STATE_INV_5;
                            state_change_reason=8; //Mains failed and solar unavailable
                            solar_cycling=0;
						}
					}	
                    /*If solar is available and solar voltage is greater than settable-hysterisis transit to state 3*/ 
                    /*settable depends on regular day or equalization day*/
					else if((algo_param.cur_bat_v < bat_sol_con_v) && \
						(algo_param.cur_sol_v > (bat_voltage_settable-equ_hys)))
					{
                        if(prevent_solar_charging==0){
                        state_change_reason=9; //Solar available and battery not full
                        solar_cycling=1;
						algo_param.dev_algo_state = ALGO_STATE_SOL_CHG_3;
                        }
					}
				}

				break;
			
            /*If battery voltage reaches maximum transit to state 1*/
            /*Settable depends on regular or equalization day*/    
			case ALGO_STATE_SOL_CHG_3:
				if((algo_param.cur_bat_v > bat_voltage_settable) || \
					(low_sol_flag == 1)||(mains_sol_chg_flag==1)||(early_ft_entry)){
                    solar_cycling=1;
                    if((algo_param.cur_bat_v > bat_max_v)||(early_ft_entry))
					{
                        prevent_solar_charging_of_device();
						bat_full_flag = 1;
					}
                    if((algo_param.cur_bat_v > bat_voltage_settable))
                    {
                        solar_cycling=1;
                        state_change_reason=10; //Battery voltage reached maximum
                        solar_equ_cnt++;
                    }
                    if((low_sol_flag == 1))
                    {
                        low_sol_flag = 0;
                        solar_cycling=1;
                        state_change_reason=11; //Low solar current exit
                    }
					algo_param.dev_algo_state = ALGO_STATE_CHARG_INHIBIT_1;
				}
				break;

            /*State 4 the load is met by battery and inverter. The load is purposefully disconnected from the mains*/
            /*State 4 can happen either normally or induced purposefully during useable soc calculation*/    
			case ALGO_STATE_BAT_FRC_TRIP_4:
                clearOverVoltageFlag();
				bat_full_flag = 0;
				if(((exception_flag == 0)||(is_time_to_cal_useable_soc))||(is_time_to_dynamic_ft_entry))
				{
                    /*During useable soc calculation, the voltage and dod is calculated for every .2 drop in voltage*/
                
                    if((previous_voltage>algo_param.cur_bat_v+0.1)&&is_time_to_cal_useable_soc)
                    {
                        load.log_SOC[load_SOC_vol_index]=(total_bat_capacity)-Bat_SOC;
                        load.log_voltage[load_SOC_vol_index]=algo_param.cur_bat_v;
                        load.log_current[load_SOC_vol_index]=algo_param.cur_dis_i;
                        previous_voltage=algo_param.cur_bat_v;
                        load_SOC_vol_index+=1;
                    }
                    
                    /*While normal operation the FT exits when battery hits frc trip exit voltage*/
                    /*While calculating useable SOC the FT exits when voltage hits (bat_vol+0.2)-1.0*/
					if((((Bat_DoD>useable_SOC1)||(algo_param.cur_bat_v < bat_frc_trip_exit_v))&&(is_time_to_cal_useable_soc==0)) || \
						(dc_over_load_flag == 1) || \
						(dc_under_load_flag == 1) || \
						(ready_for_frc_trip == 0)||((algo_param.cur_bat_v < useable_bat_frc_trip_exit_v)&&(is_time_to_cal_useable_soc==1))||(eve_dy_exit==1)||(dy_exit==1))
					{
                        if(sss_start==1)
                        {
                            sss_start=0;
                            prev_sss=0;
                            sss_start_dur=0;
                            clear_common_flag(CF_SS);
                            i2c_eeprom_write_buff(0xFE7F,(uint8_t *) &common_flag,2);
                        }
                        /*If during useable soc calculation it exits because of overload*/
                        /*Set useable soc calculation failed*/
                        if((dc_over_load_flag == 1)||(dc_under_load_flag == 1)||(ready_for_frc_trip == 0)||(dy_exit==1)||(eve_dy_exit==1))
                        {
                            
                            if(dc_over_load_flag == 1)
                            {
                                state_change_reason=12; //DC Overload
                            }
                            
                            if(dc_under_load_flag == 1)
                            {
                                state_change_reason=13; //DC Underload
                                post_system_flag(SF_INV_FAIL);
                                prev_frc_trip_count=frc_trip_count;
                                frc_trip_count = max_frc_per_day;
                            }
                            
                            if(ready_for_frc_trip == 0)
                            {
                                if(button_press)
                                {
                                    state_change_reason=14; //User button press
                                    button_press=0;
                                }
                                else
                                {
                                    state_change_reason=15; //Not ready for FT
                                }
                            }
                            
                            if(eve_dy_exit==1)
                            {
                                eve_dy_exit=0;
                                state_change_reason=25; //Evening FT Exit
                                
                            }
                            
                            if(dy_exit==1)
                            {
                                dy_exit=0;
                                state_change_reason=23; //Dynamic FT Exit
                            }
                            
                            if((is_time_to_cal_useable_soc))
                            {
                                is_time_to_cal_useable_soc=0;
                                re_cal_useable_soc_count++;
                                if(re_cal_useable_soc_count>1)
                                {
                                    enable_cal_useable_soc=0;
                                    need_eq_after_abs=0;
                                    post_system_flag(SF_USEABLE_SOC_FAILED);
                                    re_cal_useable_soc_count=0;
                                    //useable SoC Calculation failed
                                }
                                else
                                {
                                    ft_exit_start_day++;
                                    ft_entry_start_day++;
                                    post_system_flag(SF_USEABLE_SOC_CAL);
                                }
                            }
                        }
                        
                        /*If it exits because of voltage hit*/
                        if((((Bat_DoD>useable_SOC1)||(algo_param.cur_bat_v < bat_frc_trip_exit_v))&&(is_time_to_cal_useable_soc==0))||((algo_param.cur_bat_v < useable_bat_frc_trip_exit_v)&&(is_time_to_cal_useable_soc==1)))
						{
                            if(is_time_to_cal_useable_soc)
                            {
                                state_change_reason=16; //Useable SoC Calculation Exit
                            }
                            else
                            {
                                if((Bat_DoD>useable_SOC1))
                                {
                                    state_change_reason=17; //Exit due to DoD violation
                                }
                                else
                                {
                                    state_change_reason=18; //Exit due to voltage violation
                                }
                            }
                            
                            /*If calculate useable soc is set*/
                            if(is_time_to_cal_useable_soc)
                            {
                                
                                load.log_SOC[load_SOC_vol_index]=(total_bat_capacity)-Bat_SOC;
                                load.log_voltage[load_SOC_vol_index]=algo_param.cur_bat_v;
                                load.log_current[load_SOC_vol_index]=algo_param.cur_dis_i;
                                previous_voltage=algo_param.cur_bat_v;
                                load_SOC_vol_index+=1;
                                
                                /*Set the AH that the battery can deliver as the total capacity of the battery. 1.12 is accounted for the extra energy available in the battery since
                                 it is not drained to its maximum capacity*/
                                useable_SOC1=load.log_SOC[load_SOC_vol_index-1]*useable_soc_multiplier;
                                useable_SOC_written=1; 
                                
                                /*Once useable soc is calculated SF_USEABLE_SOC_RC flag is set*/
                                /*RC indicates rule change*/
                                post_system_flag(SF_USEABLE_SOC_RC);
                                                                
                                /*Log the time at which the rule change occurred*/
                                load.date_time[0] = date_time->tm_mday;   //Set day
                                load.date_time[1] = date_time->tm_mon;    //Set month
                                load.date_time[2] = date_time->tm_year;   //set year
                                load.date_time[3] = date_time->tm_hour;   //Set Hour
                                load.date_time[4] = date_time->tm_min;    //Set minute
                                
                                /*
                                 * The following code is removed
                                 * This causes early exit from FT due to current compensation value of exit voltage
                                 */
//                                /*Calculate the new exit voltage threshold value*/
//                                for(int i=load_SOC_vol_index-1;i>=0;i--)
//                                {
//                                    if(useable_SOC1>load.log_SOC[i])
//                                    {
//                                      /*Update force trip exit threshold*/  
//                                      bat_frc_trip_exit_v_thr=((((useable_SOC1-load.log_SOC[i])/(load.log_SOC[i+1]-load.log_SOC[i]))*(load.log_voltage[i+1]-load.log_voltage[i]))+load.log_voltage[i])+(0.02*((load.log_current[i+1]+load.log_current[i])/2)); 
//                                      break;
//                                    }
//                                }
//                                /*Check if the force trip exit threshold is within limits.If limits are satisfied keep the value*/
//                                /*If calculated value is more than the higher bound, set higher bound as the exit value*/
//                                /*If it is lower than the lower bound, set exit voltage as (bat_max_v/80)+(lower bound)*/
//                                
//                                if(bat_frc_trip_exit_v_thr>VFT_Exit_Higher)
//                                {
//                                    bat_frc_trip_exit_v_thr=VFT_Exit_Higher;
//                                }
//                                else
//                                {
//                                    if(bat_frc_trip_exit_v_thr<VFT_Exit_Lower)
//                                    {
//                                        bat_frc_trip_exit_v_thr = VFT_Exit_Lower;
//                                    }
//                                }
//                                
//                                bat_mains_chg_in_v_thr=bat_frc_trip_exit_v_thr-0.2;
                                
                                //Clear the useable soc calculate flag
                                clear_system_flag(SF_USEABLE_SOC_CAL);
                                is_time_to_cal_useable_soc=0;
                                need_solar_mains_top_up=1;
                                mains_sol_chg_flag = 1;
                                post_system_flag(SF_SMC);
                                useable_soc_cal_cnt+=1;
                                load_SOC_vol_index=0;
                                
                                dev_state_flag=dev_state_flag|Bit(2);

                                i2c_eeprom_write_byte(DEV_SYS_FLAG,dev_state_flag);
                                vTaskDelay(500);
                                i2c_eeprom_write_float(USEABLE_SOC,useable_SOC1);
                                vTaskDelay(500);
//                                i2c_eeprom_write_float(VFT_EXT_THR,bat_frc_trip_exit_v_thr);
//                                timer_delay_ms(500);
                                i2c_eeprom_write_buff(0xFE81,(uint8_t *) &load,245);
                            }
/*The following code has been removed from new version. 
 The reasons for omitting the code are:-
 * 1. Complexity in calculating the energy discharged by battery during force trip
 * 2. This might lead to a race condition 
 */
                            
/*Instead useable soc will be calculated once in every four months*/
                            else
                            {
                                is_time_to_cal_useable_soc=0;
                                useable_soc_cal_failed=0;
                                
                                if(is_time_to_dynamic_ft_entry)
                                {
                                    if(is_day_flag==NOW_NIGHT_TIME)
                                    {
                                        dynamic_ft_complete=1;
                                    }
                                    is_time_to_dynamic_ft_entry=0;
                                }
                                
                                if((Bat_DoD>useable_SOC1)||(algo_param.cur_bat_v < bat_frc_trip_exit_v))
                                {
                                    frc_trip_count++;
                                }
                            }
           
                        }
                        /*The force trip is paused for 30minutes. Even though the battery becomes full only after 30minutes another force trip will happen*/
                        force_trip_pause_flag = 1;
                        force_trip_pause_count = 0;
                        algo_param.dev_algo_state = ALGO_STATE_INV_5;
                        solar_cycling=0;
                        dy_exit=0;
                        if(is_time_to_dynamic_ft_entry)
                        {
                            if(is_day_flag==NOW_NIGHT_TIME)
                            {
                                dynamic_ft_complete=1;
                            }
                            is_time_to_dynamic_ft_entry=0;
                        }
					}
                    /*If solar is available during normal force trip condition the algorithm moves to solar assisted force trip*/
					else if((algo_param.cur_bat_v < bat_sol_con_v) && \
							(algo_param.cur_sol_v > bat_max_v)&&(is_time_to_cal_useable_soc==0)&&(sss_start==0))
					{
                        if(prevent_solar_charging==0){
                            if(solar_cycling==1)
                            {
                                solar_cycling=1;
                            }
                            algo_param.dev_algo_state = ALGO_STATE_BAT_SOL_FRC_TRIP_6;
                            state_change_reason=19; //Solar Available and battery not full
                        }
					}
                }
                break;
            /*If the algorithm is in solar assisted force trip and sun down happens or battery gets full it moves to normal force trip*/
			case ALGO_STATE_BAT_SOL_FRC_TRIP_6:
				if((algo_param.cur_bat_v > bat_max_v) || (low_sol_flag == 1) || (ready_for_frc_trip == 0))
				{
                    if(algo_param.cur_bat_v > bat_max_v)
                    {
                        solar_equ_cnt++;
                        solar_cycling=1;
                        state_change_reason=10; //Battery voltage reached max
                        prevent_solar_charging_of_device();
                    }
                    
                    if(low_sol_flag == 1)
                    {
                        low_sol_flag = 0;
                        state_change_reason=11; //Low solar 
                        solar_cycling=1;
                    }
					algo_param.dev_algo_state = ALGO_STATE_BAT_FRC_TRIP_4;
				}
            /*If battery voltage reaches lower limit during force trip or overload occurs the algorithm goes to state 5*/
				else if((algo_param.cur_bat_v < bat_frc_trip_exit_v) || \
					(dc_over_load_flag == 1) || \
					(dc_under_load_flag == 1) || \
					(ready_for_frc_trip == 0)||(Bat_DoD>useable_SOC1)||(dy_exit==1)||(eve_dy_exit==1))
				{
                    if(sss_start==1)
                    {
                        sss_start=0;
                        prev_sss=0;
                        sss_start_dur=0;
                        clear_common_flag(CF_SS);
                        i2c_eeprom_write_buff(0xFE7F,(uint8_t *) &common_flag,2);
                    }
                    if(algo_param.cur_bat_v < bat_frc_trip_exit_v)
                    {
                        state_change_reason=18; //Exit due to voltage violation
                    }
                    
                    if((Bat_DoD>useable_SOC1))
                    {
                        state_change_reason=17; //Exit due to SoC violation
                    }

                    if(dc_over_load_flag == 1)
                    {
                        state_change_reason=12; //Overload
                    }
                    if(dc_under_load_flag == 1)
                    {
                        state_change_reason=13; //Underload
                        post_system_flag(SF_INV_FAIL);
                        prev_frc_trip_count=frc_trip_count;
                        frc_trip_count = max_frc_per_day;
                    }
                    if(ready_for_frc_trip == 0)
                    {
                        if(button_press)
                        {
                            state_change_reason=14; //User exit
                            button_press=0;
                        }
                        else
                        {
                            state_change_reason=15; //Not Ready for FT
                        }
                    }
                    
                    if(eve_dy_exit==1)
                    {
                        eve_dy_exit=0;
                        state_change_reason=25; //Evening exit

                    }

                    if(dy_exit==1)
                    {
                        dy_exit=0;
                        state_change_reason=23; //Dynamic exit
                    }
                    
					if(algo_param.cur_bat_v < bat_frc_trip_exit_v)
					{
						frc_trip_count++;
					}
                    
					force_trip_pause_flag = 1;
					force_trip_pause_count = 0;
					algo_param.dev_algo_state = ALGO_STATE_INV_5;
                    solar_cycling=0;
                    
                    if(is_time_to_dynamic_ft_entry)
                    {
                        if(is_day_flag==NOW_NIGHT_TIME)
                        {
                            dynamic_ft_complete=1;
                        }
                        is_time_to_dynamic_ft_entry=0;
                    }
				}
				break;
            /*If the state is in solar assisted mains charging, and solar energy is low, the algorithm moves to state 5*/
            /*A feature to charge the battery using both solar and mains have to be implemented here*/
            /*If battery charging current becomes greater than limited value the algorithm should branch to state 3*/    
			case ALGO_STATE_SOL_INV_DIS_7:
				if(((algo_param.cur_bat_v > bat_max_v) || \
					(low_sol_flag == 1) ||(algo_param.cur_chg_i>sol_mains_chg_cur_thr)))
				{
					if(algo_param.cur_bat_v > bat_max_v)
					{
						bat_full_flag = 1;
                        solar_cycling=1;
					}
                    if(algo_param.cur_chg_i>sol_mains_chg_cur_thr)
                    {
                        mains_sol_chg_flag=0;
                        state_change_reason = 35; //Charging current greater than 25A
                        if(test_system_flag(SF_SMC))
                        {
                            clear_system_flag(SF_SMC);
                        }
                        algo_param.dev_algo_state = ALGO_STATE_SOL_CHG_3;
                        solar_cycling=0;
                    }
                    else
                    {
                        if(algo_param.cur_bat_v > bat_max_v)
                        {
                            prevent_solar_charging_of_device();
                            if(eve_soc_error==1)
                            {
                               mains_sol_chg_flag=1;
                            }
                            else
                            {
                               mains_sol_chg_flag=0; 
                            }                          
                            if(test_system_flag(SF_SMC))
                            {
                                clear_system_flag(SF_SMC);
                            }
                            state_change_reason=10; //Bat full
                        }
                        if(low_sol_flag == 1)
                        {
                            low_sol_flag = 0;
                            state_change_reason=11; //Low solar
                            solar_cycling=1;
                        }
//                        mains_chg_flag=0;
                        algo_param.dev_algo_state = ALGO_STATE_INV_5;
                    }
					
				}
				break;

			default:
				algo_param.dev_algo_state = ALGO_STATE_INV_5;
				break;		
		}
        
        if((total_time>(((uint32_t) Previous_Day_Sundown *(uint32_t) 3600)-(uint32_t)5400))&&(eve_soc_error==0)&&(is_day_flag == NOW_DAY_TIME)&&(is_time_to_cal_useable_soc==0)&&(dynamic_ft_complete==0))
        {
            /*This is to ensure battery is greater than 90% of total capacity during sun down*/
            if(Bat_SOC<(eve_soc_corr*total_bat_capacity))    
            {
                mains_sol_chg_flag = 1;
                post_system_flag(SF_SMC);
                eve_soc_error=1; 
                post_system_flag(EVE_SOC_ERR);
            }
            else
            {
                eve_soc_error=0;
            }
        }
        
#if 1
    if(test_common_flag(CF_SS)&&(total_time>sss_time)&&(sss_start==0))
    {
        if(is_time_to_cal_useable_soc==1)
        {
            is_time_to_cal_useable_soc=0;
        }
        if(is_time_to_dynamic_ft_entry)
        {
            if(is_day_flag==NOW_NIGHT_TIME)
            {
                dynamic_ft_complete=1;
            }
            is_time_to_dynamic_ft_entry=0;
        }
        
        switch(sss)
        {
            case ALGO_STATE_INV_5:
                 switch_state_4_5();
                 state_change_reason=28; //Server set state 5
                 sss_start=1;
                 prev_sss=sss;
                 if(is_day_flag==NOW_NIGHT_TIME){
                    no_inh_flag=1; 
                 }
                 break;
            case ALGO_STATE_CHARG_INHIBIT_1:
                 algo_param.dev_algo_state = ALGO_STATE_CHARG_INHIBIT_1;
                 state_change_reason=29; //Server set state 1
                 no_frc_trip_flag = 1;
                 sss_start=1;
                 prev_sss=sss;
                 break;
            case ALGO_STATE_BAT_FRC_TRIP_4:
                 algo_param.dev_algo_state = ALGO_STATE_BAT_FRC_TRIP_4;
                 sss_start=1;
                 prev_sss=sss;
                 state_change_reason=31; //Server set state 4
                 prev_ft_count=frc_trip_count;
                 force_trip_pause_count = 0;
				 force_trip_pause_flag = 0;
                 ready_for_frc_trip = 1;
                 if(is_day_flag==NOW_NIGHT_TIME){
                    is_time_to_dynamic_ft_entry=1; 
                 }
                 break;
            case ALGO_STATE_SOL_CHG_3:
                 if(is_day_flag==NOW_DAY_TIME){
                  algo_param.dev_algo_state = ALGO_STATE_SOL_CHG_3;
                  sss_start=1;
                  state_change_reason=30; //Server set state 3
                  no_frc_trip_flag = 1;
                  prev_sss=sss;              
                  break;
                 }
                 else{
                    clear_common_flag(CF_SS);
                    i2c_eeprom_write_buff(0xFE7F,(uint8_t *) &common_flag,2);
                 }
                 break;
            case ALGO_STATE_BAT_SOL_FRC_TRIP_6:
                 if(is_day_flag==NOW_DAY_TIME){
                    algo_param.dev_algo_state = ALGO_STATE_BAT_SOL_FRC_TRIP_6;
                    sss_start=1;
                    state_change_reason=32; //Server set state 6
                    prev_sss=sss;
                    prev_ft_count=frc_trip_count;
                    frc_trip_count=0;
                    force_trip_pause_count = 0;
                    force_trip_pause_flag = 0;
                 }
                 else{
                    clear_common_flag(CF_SS);
                    i2c_eeprom_write_buff(0xFE7F,(uint8_t *) &common_flag,2);                  
                 }
                 break;
            case ALGO_STATE_SOL_INV_DIS_7:
                 if(is_day_flag==NOW_DAY_TIME){
                    // switch_state_4_7();
                    state_change_reason=33; //server set state 7
                    sss_start=1;
                    prev_sss=sss;
                    break;
                 }
                 else{
                    clear_common_flag(CF_SS);
                    i2c_eeprom_write_buff(0xFE7F,(uint8_t *) &common_flag,2);                     
                 }
        }
        state_change_status = 1;
        one_sec_log_time_out = 0;
    }
    
    if(sss_start==1)
    {
        if(++sss_start_dur>sssd)
        {
            sss_start_dur=0;
            sss_start=0;
            clear_common_flag(CF_SS);
            i2c_eeprom_write_buff(0xFE7F,(uint8_t *) &common_flag,2);
            state_change_reason=34; //server set time exit
            switch(prev_sss)
            {
                case ALGO_STATE_INV_5:
                     switch_state_4_5();
                     if(is_day_flag==NOW_NIGHT_TIME)
                     {
                        no_inh_flag=0;
                     }
                     break;
                case ALGO_STATE_CHARG_INHIBIT_1:
                     no_frc_trip_flag = 0;
                     break;
                case ALGO_STATE_BAT_FRC_TRIP_4:
                     algo_param.dev_algo_state = ALGO_STATE_CHARG_INHIBIT_1;
                     frc_trip_count=prev_ft_count;
                     if(is_day_flag==NOW_NIGHT_TIME)
                     {
                       is_time_to_dynamic_ft_entry=0; 
                     }
                     break;
                case ALGO_STATE_SOL_CHG_3:
                     no_frc_trip_flag = 0;
                     break;
                case ALGO_STATE_BAT_SOL_FRC_TRIP_6:
                     algo_param.dev_algo_state = ALGO_STATE_CHARG_INHIBIT_1;
                     frc_trip_count=prev_ft_count;
                     break;
                case ALGO_STATE_SOL_INV_DIS_7:
                     algo_param.dev_algo_state = ALGO_STATE_INV_5;
                     break;
            }
            prev_sss=0;
            state_change_status = 1;
            one_sec_log_time_out = 0;
        }
    }
#endif
        /*Set the switch state*/
		turtle_set_sw(algo_param.dev_algo_state);
                
        if(one_sec_log_time_out==0)
        {
            if((prev_dev_state!=algo_param.dev_algo_state)&&(solar_cycling==0))
            {
                is_time_to_log=1;
                state_change_status=1;
            }
        }
        
        if(state_change_status)
        {
            one_sec_log_time_out++;
            if(one_sec_log_time_out>state_change_log_interval_duration_seconds)
            {
                is_time_to_log=0;
                one_sec_log_time_out=0;
                state_change_status=0;
            }
            else
            {
                is_time_to_log=1;
            }
        }
        /*To note the previous state*/
        prev_dev_state=algo_param.dev_algo_state;
	}
    else
    {
        if(algo_param.dev_algo_state != ALGO_STATE_INV_5)
        {
            if(is_sol_sensed==0)
            {
                state_change_reason=21; //Solar disconnected
            }
            else
            {
                state_change_reason=22; //PAC Error
            }
            switch_state_4_5();
            turtle_set_sw(algo_param.dev_algo_state);
        }
        clearOverVoltageFlag();
    }

	if((algo_param.dev_algo_state == ALGO_STATE_BAT_SOL_FRC_TRIP_6) ||(algo_param.dev_algo_state == ALGO_STATE_SOL_INV_DIS_7) ||(algo_param.dev_algo_state == ALGO_STATE_SOL_CHG_3))
	{
		bat_equ_param.equ_sol_sw_stat = 1;
	}
	else
	{
		bat_equ_param.equ_sol_sw_stat = 0;
	}
    
    if(solar_cycling)
    {
        if((algo_param.dev_algo_state == ALGO_STATE_BAT_SOL_FRC_TRIP_6) ||(algo_param.dev_algo_state == ALGO_STATE_SOL_INV_DIS_7) ||(algo_param.dev_algo_state == ALGO_STATE_SOL_CHG_3))
        {
             cycling_energy=(float)(pac_sol_p/3600);
             cycling_solar_present_time++;
        }
        else
        {
            cycling_solar_absent_time++;
        }
    }
	exception_flag = 0;
    // WDT_Fcn();
	turtle_set_sw(algo_param.dev_algo_state);
    show_led();
}

void
turtle_algorithm()
{
    ESP_LOGI(TAG,"Inside the turtle algorithm");
    show_led();
    // WDT_Fcn(); 
	check_time();
    if((sunup_conf_enable==0x01)&&(is_day_flag == NOW_NIGHT_TIME)){
        if(++timetosunup>=300){
           readyforsunup=0x01;
        }
    }
    xSemaphoreTake(i2cSemaphore,portMAX_DELAY);
	if(get_sensor_values(MV_AVG_ADC_VAL,1) == 0){
		if(++pac_error_flag_cnt>3){
            pac_error_flag_cnt = 0;
		    pac_error_flag = 1;
        }
	}
    else{
        pac_error_flag = 0;
        pac_error_flag_cnt = 0;
    }
    xSemaphoreGive(i2cSemaphore);

#if 1    
    if(mains_sense()){
        ESP_LOGI(TAG,"Mains sense High");
        if(++mains_sense_dur>2){
            mains_available=1;
        }
    }
    else{
        ESP_LOGI(TAG,"Mains sense Low");
        mains_sense_dur = 0;
        mains_available=0;
    }
#endif

	calc_summary();

    algo_interval_count++;

    if(prevent_solar_charging) {
        if (++prevent_solar_charging_timeout>5){
            /* code */
            prevent_solar_charging_timeout=0;
            prevent_solar_charging=0;
        }
    }

    if((sunup_conf_enable==0x01)&&(is_day_flag == NOW_NIGHT_TIME))
    {
       if(readyforsunup==0)
       {
           algo_param.cur_sol_v = 0.0;
       }
    }
    //Increment solar seconds
    solar_time_seconds++;
    //After one hour increment solar time
    if(solar_time_seconds>=3600)
    {
        //Reset solar seconds
        solar_time_seconds=0;
        //Increment solar time
        Solar_Time++;
        //If solar time is greater than 23
        if(Solar_Time>23)
        {
            //Reset solar time
            Solar_Time=23;
        }
    }

    /* No.of Modules Check*/
    /*If solar current is more than 40A stop solar charging *
     * This check is done to ensure that the no.of modules connected is not
     *  more than the requirement. Set the system to state 5*/
    if(algo_param.cur_sol_i>max_solar_current) {
        is_time_to_log=1;
        stop_solar_chg=1;   
    }
    
    //bucket load power
    load_power_bucket[Solar_Time]+=(float)((pac_sol_p-sol_bat_p)/3600)+(pac_bat_dis_p/3600)+(ac_load_p/3600);
    
    if(device_state==ALGO_ON)
    {
            if(stop_solar_chg==1)
    		{
                if(++stop_solar_chg_cnt>1800)
                {
                    stop_solar_chg=0;
                }
                
        		if(algo_param.dev_algo_state != ALGO_STATE_INV_5)
        		{
                    state_change_reason=26; //High solar current
            		switch_state_4_5();
        		}
    		}
    		else{                      
    			if(Solar_Time<13){
               		solar_power_bucket[Solar_Time]+=(float)(pac_sol_p/3600);
            	}
            	Today_Recieved_Solar_Energy+=(float)(pac_sol_p/3600);
                state_change_algorithm();
            }
    }
    else
    {
        clearOverVoltageFlag();
        if(algo_param.dev_algo_state!=ALGO_STATE_INV_5)
        {
            switch_state_4_5(); 
        }
        show_led();
    }

    algo_param.cur_inv_v=inverter_vol;
    algo_param.state_change_reason=state_change_reason;
    algo_param.dev_stat=system_flag;
    algo_param.algo_stat=state_flag|(user_device_off<<15);
    algo_param.error=error_flag;
	if((algo_interval_count >= DEBUG_LOG_INTERVAL_DUR_SEC)){
		is_time_to_log = 1;
		algo_interval_count = 0; 
	}
    
    if(is_time_to_log){
		if(!test_error(ERR_RTC) && !test_error(ERR_NO_MEMORY)){
    		if(prev_hour != date_time->tm_hour){
                ESP_LOGI(TAG,"Logging 1 hour data");
                // https_req_buff_write(&https_request_buffer_body,get_information);
	    		algo_param.log_type = 1;
	    		log_to_eeprom();
	    		one_hour_log_flag = 1;
	    		clear_summary();
	    	}
	    	if(turtle_log_available() > CONT_LOG_STOP_CNT){
    			debug_log_flag = 0;
                state_change_log_interval_duration_seconds=(uint8_t) i2c_eeprom_read_byte(STATE_CHANGE_LOG_UNITS_NO_WIFI);
    		}
    		else{
    			debug_log_flag = 1;
                state_change_log_interval_duration_seconds=(uint8_t) i2c_eeprom_read_byte(STATE_CHANGE_LOG_UNITS);
    		}
            if(state_change_status){
                algo_param.log_type=4;
                xSemaphoreTake(i2cSemaphore,portMAX_DELAY);
                log_to_eeprom();
                xSemaphoreGive(i2cSemaphore);
            }
            else if(debug_log_flag){
	    		algo_param.log_type = 3;
                xSemaphoreTake(i2cSemaphore,portMAX_DELAY);
	    		log_to_eeprom();
                xSemaphoreGive(i2cSemaphore);
    		}
    	}	    		
		is_time_to_log = 0;
    }

}

void calculate_utility_savings(void){
    if((algo_param.dev_algo_state!=5)&&(algo_param.dev_algo_state!=7)){
        if(dev_type==1){ algo_param.utility_savings += utility_saving_array[0]; };
        if(dev_type==2){ algo_param.utility_savings += utility_saving_array[1]; };
        if(dev_type==5){ algo_param.utility_savings += utility_saving_array[2]; };
        if(dev_type==6){ algo_param.utility_savings += utility_saving_array[3]; };
    }
    ESP_LOGI(TAG,"Utility savings is %f", algo_param.utility_savings);
}

void
calc_summary(){
    //The discharge current for last 3minutes is stored in an array
    absorption_current[abs_index]=algo_param.cur_dis_i;
    //Increment the element address
    abs_index += 1;
    //If the array gets filled the last value is stored in the first element of the array
    if(abs_index>=180){
        abs_index=0;
    }
    //The SOC of the battery is calculated by adding the previous SOC with the battery charging current  and subtracting the total with the discharge current
    //The battery charging current and discharge currents are converted into AH by dividing it by 3600
    Bat_SOC = Bat_SOC+(algo_param.cur_bat_v*(algo_param.cur_chg_i-algo_param.cur_dis_i)/3600);
    //If the AH goes beyond the rated battery AH the rated AH is set as the current SOC
    //SOC is calculated in terms of AH
    Bat_DoD = total_bat_capacity - Bat_SOC;
    
    if(Bat_SOC>(total_bat_capacity))
    {
        Bat_SOC=total_bat_capacity; //If SOC goes beyond rated AH it is reset
    }
    
    if(Bat_SOC<(0.5*total_bat_capacity))
    {
        battery_low = 1;
    }
    else
    {
        battery_low = 0;
    }
    
    calculate_utility_savings();
    //Before sending to the server the SOC is converted into percentage of the rated AH
    algo_param.bat_cur_cap_coul = (Bat_SOC/(total_bat_capacity))*100;  
	algo_param.sol_e += pac_sol_p;
	algo_param.sol_bat_e += sol_bat_p;
	algo_param.bat_dis_e += pac_bat_dis_p;
	algo_param.bat_ac_chg_e += mains_chg_p;
	algo_param.ac_load_e += ac_load_p;
	algo_param.num_sum_samples++;
}



void 
clear_summary()
{
	algo_param.sol_e = 0;
	algo_param.sol_bat_e = 0;
	algo_param.bat_dis_e = 0;
	algo_param.bat_ac_chg_e = 0;
	algo_param.ac_load_e = 0;
	algo_param.num_sum_samples = 0;	
	prev_hour = date_time->tm_hour;
    algo_param.utility_savings = 0.0;
    https_req_buff_write(&https_request_buffer_body,get_information);
//#warning "load previous hour here"	
}

void check_battery_voltage(){
    float batVol;
    xSemaphoreTake(i2cSemaphore,portMAX_DELAY);
    batVol = getBatteryVoltage();
    xSemaphoreGive(i2cSemaphore);
    
    if(  batVol > bat_max_v ){
        
        ESP_LOGI(TAG,"Battery value above set threshold");
        prevent_solar_charging_of_device();
        solar_equ_cnt+=1;
        bat_full_flag = 1; //Battery full flag status set
        Bat_SOC=total_bat_capacity;
        exception_flag = 1; //Exception flag for data log
        if(algo_param.dev_algo_state == ALGO_STATE_BAT_SOL_FRC_TRIP_6){ //If state is 6 change to 4
            algo_param.dev_algo_state = ALGO_STATE_BAT_FRC_TRIP_4;
        }
        else if(algo_param.dev_algo_state == ALGO_STATE_SOL_INV_DIS_7){ //If state is 7 change to 5
            mains_sol_chg_flag=0;
            if(test_system_flag(SF_SMC))
            {
                clear_system_flag(SF_SMC);
            }
            algo_param.dev_algo_state = ALGO_STATE_INV_5;
        }
        else if(algo_param.dev_algo_state == ALGO_STATE_SOL_CHG_3){ //If state is 3 change to 1
            algo_param.dev_algo_state = ALGO_STATE_CHARG_INHIBIT_1;
        }
        bat_equ_param.equ_sol_sw_stat = 0;
        sol_mos_sw_off();
        ESP_LOGI(TAG,"Solar switch off");
        solar_cycling=1;          
    }      
}


/** 
 ** Initialize the algorithm
 ** Read calibration constants from memory
 ** Flush the moving average buffer
 ** Arguments :
 **  None
 ** Reurns :
 **  None
**/
void init_algo(){
    i2c_eeprom_read_buff(EE_DEV_SER_NO_ADDR,(uint8_t *)dev_id,DEV_SER_NO_LEN);
	algo_interval_count = 0;
	force_trip_pause_flag = 0;
	force_trip_pause_count = 0;
	power_trip_flag = 0;
	bat_sol_equ_intr_cnt = 0;
    bat_sol_equ_dur_cnt = 0;
    no_sol_check_count = 0;
    is_time_to_equ = 0;
	is_sol_sensed = 0;
	no_inh_flag = 0;
	peak_inh_flag = 0;
	no_frc_trip_flag = 0;
	is_day_flag = NOW_NIGHT_TIME;
	low_sol_flag = 0; 
	dc_over_load_flag = 0;
	dc_under_load_flag = 0;
	low_sol_check_count = 0;
	sol_v_chk_count = 0;
	over_load_check_count = 0;
	under_load_check_count = 0;
	need_solar_mains_top_up = 0;
	mains_topup_count = 0;
	ready_for_frc_trip = 0;
	mains_chg_flag = 0;
	pac_error_flag = 0;
    pac_error_flag_cnt = 0;
	exception_flag = 0;
	frc_trip_count = 0;
	bat_full_flag = 0;
    debug_log_flag = 1;
    is_time_to_log = 0;
    algo_interval_count = 0; 
    soc_reset_flag = 0;
    algo_param.bat_cur_cap_coul = 0;
    algo_param.cur_temp = 0;
    init_pac1720(); //Initialize PAC
	load_sensor_constants();
	get_sensor_values(FLUSH_ADC_VAL, 0);
	algo_param.dev_algo_state = ALGO_STATE_INV_5;
	turtle_set_sw(algo_param.dev_algo_state);
	clear_summary();
	bat_equ_param.equ_sol_sw_stat = 0;
    Previous_Day_Sundown=12;
    Previous_Day_Morning_Sunhour=6;
    reset_parameters();
}



void load_sensor_constants(){
	pac_bat_v_const = i2c_eeprom_read_float(EE_PAC_BAT_V_CONST_ADDR);
	uc_bat_v_const = i2c_eeprom_read_float(EE_UC_BAT_V_CONST_ADDR);
	pac_bat_chg_i_const = i2c_eeprom_read_float(EE_PAC_BAT_CHG_I_CONST_ADDR);
    pac_bat_dis_i_const = i2c_eeprom_read_float(EE_PAC_BAT_DIS_I_CONST_ADDR);
    pac_bat_chg_i_offset = i2c_eeprom_read_float(EE_PAC_BAT_CHG_I_OFF_ADDR);
    pac_bat_dis_i_offset = i2c_eeprom_read_float(EE_PAC_BAT_DIS_I_OFF_ADDR);
	pac_bat_p_const = i2c_eeprom_read_float(EE_PAC_BAT_P_CONST_ADDR);
	pac_sol_v_const = i2c_eeprom_read_float(EE_PAC_SOL_V_CONST_ADDR);
	uc_sol_v_const = i2c_eeprom_read_float(EE_UC_SOL_V_CONST_ADDR);
	pac_sol_i_const = i2c_eeprom_read_float(EE_PAC_SOL_I_CONST_ADDR);
    pac_sol_i_offset = i2c_eeprom_read_float(EE_PAC_SOL_I_OFF_ADDR);
	pac_sol_p_const = i2c_eeprom_read_float(EE_PAC_SOL_P_CONST_ADDR);
	ct_load_i_offset = i2c_eeprom_read_int16(EE_CT_LOAD_I_OFST_ADDR);
	ct_load_i_const = i2c_eeprom_read_float(EE_CT_LOAD_I_CONST_ADDR);
    uc_temp1_const = i2c_eeprom_read_float(EE_UC_TEMP1_CONST_ADDR);
    uc_temp2_const = i2c_eeprom_read_float(EE_UC_TEMP2_CONST_ADDR);
	max_dc_load_i = i2c_eeprom_read_float(EE_DC_OVER_I_THR_ADDR);
	max_ac_load_i = i2c_eeprom_read_float(EE_AC_OVER_I_THR_ADDR);
    battery_type = i2c_eeprom_read_byte(EE_BAT_TYPE_ADDR);
    dev_type=i2c_eeprom_read_byte(EE_DEVICE_TYPE_ADDR);   
	bat_max_v = i2c_eeprom_read_float(EE_BAT_MAX_VOLT_ADDR);
    extern PPAC194X5X_DEVICE_CONTEXT pPACdevice;

    if((dev_type==1)||(dev_type==5)){
        PAC194x5x_SetOverVolt_limit(pPACdevice, 15.0);
    }
    else{
        if((dev_type==2)||(dev_type==6)){
            PAC194x5x_SetOverVolt_limit(pPACdevice, 29.0);
        }
    }
    
    Rated_Bat_AH = i2c_eeprom_read_float(EE_BAT_CAP_ADDR);
    bat_age = i2c_eeprom_read_byte(EE_BAT_AGE_ADDR);
    dev_state_flag=i2c_eeprom_read_byte(DEV_SYS_FLAG);
    eve_soc_corr=i2c_eeprom_read_float(EVE_BAT_SOC_COR_LOW_LIM);
    night_soc_corr=i2c_eeprom_read_float(EVE_BAT_SOC_COR_HIGH_LIM);
    if(dev_state_flag&Bit(2)){    
        bat_frc_trip_exit_v_thr = i2c_eeprom_read_float(VFT_EXT_THR);
    }
    else{
        bat_frc_trip_exit_v_thr = i2c_eeprom_read_float(EE_BAT_FRC_EXIT_VOLT_ADDR);
    }
    bat_vol=i2c_eeprom_read_float(EE_BAT_FRC_EXIT_VOLT_ADDR);
    max_frc_per_day = i2c_eeprom_read_byte(EE_FT_NUM_ADDR);
    total_bat_capacity=bat_vol*Rated_Bat_AH;
    sol_inst = i2c_eeprom_read_float(EE_SOL_CAP)*1000.0;
    if(dev_state_flag&Bit(2)){
        bat_mains_chg_in_v_thr = bat_frc_trip_exit_v_thr-0.2;    
	}
    else{
        bat_mains_chg_in_v_thr = i2c_eeprom_read_float(EE_BAT_MAINS_CHG_IN_VOLT_ADDR);
    }
    if(dev_state_flag&Bit(1)){
        weight1=(float)i2c_eeprom_read_float(DAILY_LOAD_THR);
        weight2=1.0-weight1;
    }
    else{
        if(dev_state_flag&Bit(2)){
            weight1=(float)i2c_eeprom_read_float(WEEKLY_LOAD_THR);
            weight2=1.0-weight1;
        }
        else{
            weight1=0.75;
            weight2=1.0-weight1;
        }
    }
    sss=i2c_eeprom_read_byte(SER_ASSERT_STATE);
    sssd=i2c_eeprom_read_int16(SER_STATE_ASSERTED_DUR);
    sath=i2c_eeprom_read_byte(SER_STATE_ASSERTED_TIM_HR);
    satm=i2c_eeprom_read_byte(SER_STATE_ASSERTED_TIM_MIN);
    common_flag|=i2c_eeprom_read_int16(COMMON_THR);
    sss_time=(uint32_t)((uint32_t)(satm*60)+(uint32_t)(sath*3600));
    bat_mains_chg_out_v = i2c_eeprom_read_float(EE_BAT_MAINS_CHG_OUT_VOLT_ADDR);
	morn_chg_inh_flag = i2c_eeprom_read_byte(EE_MORN_CHG_INH_EN_ADDR);
	even_chg_inh_flag = i2c_eeprom_read_byte(EE_EVEN_CHG_INH_EN_ADDR);
    bat_abs_intr = i2c_eeprom_read_int16(EE_ABS_INTR_ADDR);
	bat_sol_equ_intr = i2c_eeprom_read_byte(EE_BAT_EQU_INTR_ADDR);
	bat_sol_equ_dur = (uint16_t) i2c_eeprom_read_byte(EE_BAT_EQU_DUR_ADDR) * (uint16_t) 3600;
	bat_max_volt_adc = 	(uint16_t)(bat_max_v/uc_bat_v_const);
	bat_sol_con_v = (bat_max_v - SOL_RECON_OFFSET);
    state_change_log_interval_duration_seconds=i2c_eeprom_read_byte(STATE_CHANGE_LOG_UNITS);
    useabe_SoC_start_time = (uint8_t) i2c_eeprom_read_byte(USEABLE_SOC_STRT);
    max_solar_current = (float) i2c_eeprom_read_float(SOL_MAX_CUR);
    ft_entry_start_day=i2c_eeprom_read_byte(FT_ENTRY_STRT_DAY);
    ft_exit_start_day=i2c_eeprom_read_byte(FT_EXIT_STRT_DAY);
    useable_soc_recal_dur=180;
    VFT_Exit_Higher=bat_vol*i2c_eeprom_read_float(VFT_EXIT_HIGH);
    VFT_Exit_Lower=bat_vol*i2c_eeprom_read_float(VFT_EXIT_LOW);
    abs_mains_fail_tolerance=(uint16_t)i2c_eeprom_read_byte(DEV_ABS_MF_PER)*(uint16_t)3600;
    equ_fail_tolerance=(uint16_t)i2c_eeprom_read_byte(EQ_MF_PER)*(uint16_t)3600;
    if(dev_state_flag&Bit(2)){
        useable_SOC1=i2c_eeprom_read_float(USEABLE_SOC);
    }
    else{
        useable_SOC1=(total_bat_capacity)/(2.0*(1.0+bat_age*0.5));
    }
    useable_soc_multiplier=i2c_eeprom_read_float(DEV_USEABLE_SOC_MULTIPLIER);
    equ_hys=(float)((float)bat_vol/((float)i2c_eeprom_read_byte(DEV_EQU_HYS)));
    useable_bat_frc_trip_exit_v=bat_vol-(bat_vol/15.0);
    bat_voltage_settable=bat_max_v;
    server_ft_exit_time=i2c_eeprom_read_int16(FT_EXIT_TIM);
    
    if(test_common_flag(CF_SOL_CR)){
        MA_Morning_Solar_Bucket=(float) i2c_eeprom_read_float(MORN_SOL_PRED);
        MA_Afternoon_Solar_Bucket=(float) i2c_eeprom_read_float(EVEN_SOL_PRED);
        clear_common_flag(CF_SOL_CR);
        i2c_eeprom_write_buff(0xFE7F,(uint8_t *) &common_flag,2);
    }
    
    if(test_common_flag(CF_SOL_E_UPD)){
        Day_Morning_Solar_Bucket[0]=(float) i2c_eeprom_read_float(DAY1_MORN_SOL);
        Day_Morning_Solar_Bucket[1]=(float) i2c_eeprom_read_float(DAY2_MORN_SOL);
        Day_Morning_Solar_Bucket[2]=(float) i2c_eeprom_read_float(DAY3_MORN_SOL);
        MA_Morning_Solar_Bucket=(Day_Morning_Solar_Bucket[0]+Day_Morning_Solar_Bucket[1]+Day_Morning_Solar_Bucket[2])/3;
        Day_Afternoon_Solar_Bucket[0]=(float) i2c_eeprom_read_float(DAY1_AFT_SOL);
        Day_Afternoon_Solar_Bucket[1]=(float) i2c_eeprom_read_float(DAY2_AFT_SOL);
        Day_Afternoon_Solar_Bucket[2]=(float) i2c_eeprom_read_float(DAY3_AFT_SOL);
        MA_Afternoon_Solar_Bucket=(Day_Afternoon_Solar_Bucket[0]+Day_Afternoon_Solar_Bucket[1]+Day_Afternoon_Solar_Bucket[2])/3;
        clear_common_flag(CF_SOL_E_UPD);
        i2c_eeprom_write_buff(0xFE7F,(uint8_t *) &common_flag,2);
    }
    
    if(test_common_flag(CF_LOAD_E_UPD))
    {
        Day_Morning_Load_Bucket[0]=(float) i2c_eeprom_read_float(DAY1_MORN_LOAD);
        Day_Morning_Load_Bucket[1]=(float) i2c_eeprom_read_float(DAY2_MORN_LOAD);
        Day_Morning_Load_Bucket[2]=(float) i2c_eeprom_read_float(DAY3_MORN_LOAD);
        Day_Morning_Load_Bucket[3]=(float) i2c_eeprom_read_float(DAY4_MORN_LOAD);
        Day_Morning_Load_Bucket[4]=(float) i2c_eeprom_read_float(DAY5_MORN_LOAD);
        Day_Morning_Load_Bucket[5]=(float) i2c_eeprom_read_float(DAY6_MORN_LOAD);
        Day_Morning_Load_Bucket[6]=(float) i2c_eeprom_read_float(DAY7_MORN_LOAD);
        
        Day_Night_Load_Bucket[0]=(float) i2c_eeprom_read_float(DAY1_NGT_LOAD);
        Day_Night_Load_Bucket[1]=(float) i2c_eeprom_read_float(DAY2_NGT_LOAD);
        Day_Night_Load_Bucket[2]=(float) i2c_eeprom_read_float(DAY3_NGT_LOAD);
        Day_Night_Load_Bucket[3]=(float) i2c_eeprom_read_float(DAY4_NGT_LOAD);
        Day_Night_Load_Bucket[4]=(float) i2c_eeprom_read_float(DAY5_NGT_LOAD);
        Day_Night_Load_Bucket[5]=(float) i2c_eeprom_read_float(DAY6_NGT_LOAD);
        Day_Night_Load_Bucket[6]=(float) i2c_eeprom_read_float(DAY7_NGT_LOAD);
        
        clear_common_flag(CF_LOAD_E_UPD);
        i2c_eeprom_write_buff(0xFE7F,(uint8_t *) &common_flag,2);
    }
    
    if(test_common_flag(CF_DAY))
    {
        num_days = (uint16_t) i2c_eeprom_read_int16(NUM_DAYS);
        clear_common_flag(CF_DAY);
        i2c_eeprom_write_buff(0xFE7F,(uint8_t *) &common_flag,2);   
    }

}

void reset_parameters(void){
        extern uint8_t stopBatteryOverVoltageProtectionOnCalibration;
        stopBatteryOverVoltageProtectionOnCalibration=0;
        early_ft_entry=0;
        enable_early_ft_entry=0;
        battery_type = i2c_eeprom_read_byte(EE_BAT_TYPE_ADDR);
        num_days=0;
        is_day_flag = NOW_NIGHT_TIME;
        Solar_Time=0;
        solar_time_seconds=0;
        total_time=0;
        system_flag=0;
        state_flag=0;
        prevent_solar_charging=0;
        prevent_solar_charging_timeout=0;
        for(int i=0;i<12;i++)
        {
            solar_power_bucket[i]=0.0;
        }
        Today_Recieved_Solar_Energy=0.0;
        for(int i=0;i<24;i++)
        {
            load_power_bucket[i]=0.0;
        }
                
        for(int i=0;i<20;i++)
        {
            load.log_SOC[i]=0.0;
            load.log_voltage[i]=0.0;
            load.log_current[i]=0.0;
        }
        for(int i=0;i<5;i++)
        {
            load.date_time[i]=0;
        }
        Bat_SOC=0.0;
        estimated_ft_entry_time=0;
        check_ft_entry_time=0;      
        /* If useable SoC is calculated already we skip the initialization routine */ 
        if(!(dev_state_flag&Bit(2)))
        {
            if(bat_age==0)
            {
                post_system_flag(SF_ABS_NEEDED);  
            }
            else
            {
                post_system_flag(SF_USEABLE_SOC_CAL); 
            }
        }
        else
        {
            post_system_flag(SF_ABS_NEEDED);
        }
        Previous_Day_Morning_Sunhour=6;
        Previous_Day_Sundown=12;
        soc_reset_flag = 0;
        
        if((is_time_to_cal_useable_soc))
        {
            frc_trip_count=0;
            is_time_to_cal_useable_soc=0;
            re_cal_useable_soc_count++;
            if(re_cal_useable_soc_count>1)
            {
                enable_cal_useable_soc=0;
                need_eq_after_abs=0;
                post_system_flag(SF_USEABLE_SOC_FAILED);
                re_cal_useable_soc_count=0;
            }
            else
            {
                ft_exit_start_day++;
                ft_entry_start_day++;
                post_system_flag(SF_USEABLE_SOC_CAL);
            }
        }
}

/** 
 ** When a single press detectected exit from forcetrip
 ** and will not try for a forcetrip for next 30 min
 ** Arguments :
 **  None
 ** Reurns :
 **  None
**/
void
pause_force_trip(){
	if((algo_param.dev_algo_state == ALGO_STATE_BAT_FRC_TRIP_4) || \
		(algo_param.dev_algo_state == ALGO_STATE_BAT_SOL_FRC_TRIP_6))
	{
		// algo_param.dev_algo_state = ALGO_STATE_INV_5;
        if(is_time_to_dynamic_ft_entry)
        {
            if(is_day_flag==NOW_NIGHT_TIME)
            {
                dynamic_ft_complete=1;
            }
            is_time_to_dynamic_ft_entry=0;
        }
		// turtle_set_sw(algo_param.dev_algo_state);
	}

	force_trip_pause_flag = 1;
	force_trip_pause_count = 0;
	frc_trip_count++;
}

void singlePressEffectOnAlgorithm(void){
  state_change_reason=Bit(6);
  if(test_system_flag(SF_INV_FAIL)){
    clear_system_flag(SF_INV_FAIL);
  }
  if((is_time_to_cal_useable_soc)){
      frc_trip_count=0;
      is_time_to_cal_useable_soc=0;
      re_cal_useable_soc_count++;
      if(re_cal_useable_soc_count>1){
          enable_cal_useable_soc=0;
          need_eq_after_abs=0;
          post_system_flag(SF_USEABLE_SOC_FAILED);
          re_cal_useable_soc_count=0;
      }
      else{
          ft_exit_start_day++;
          ft_entry_start_day++;
          post_system_flag(SF_USEABLE_SOC_CAL);
      }
  }
  beep(1,SHORT_BEEP);
  pause_force_trip();
  beep(1,LONG_BEEP);
}

void Solar_Energy_Bucketing(){
        Morning_Solar_Bucket=0.0;
        for(int i=0;i<Morning_Sunhour;i++){    
            Morning_Solar_Bucket=Morning_Solar_Bucket+solar_power_bucket[i];
        }      
        Afternoon_Solar_Bucket=0.0;
        for(int i=Morning_Sunhour;i<=Sundown_Time;i++){    
            Afternoon_Solar_Bucket=Afternoon_Solar_Bucket+solar_power_bucket[i];
        }
        Day_Morning_Solar_Bucket[0]=Day_Morning_Solar_Bucket[1];
        Day_Morning_Solar_Bucket[1]=Day_Morning_Solar_Bucket[2];
        Day_Morning_Solar_Bucket[2]=Morning_Solar_Bucket;
        MA_Morning_Solar_Bucket=(Day_Morning_Solar_Bucket[0]+Day_Morning_Solar_Bucket[1]+Day_Morning_Solar_Bucket[2])/3;
        Day_Afternoon_Solar_Bucket[0]=Day_Afternoon_Solar_Bucket[1];
        Day_Afternoon_Solar_Bucket[1]=Day_Afternoon_Solar_Bucket[2];
        Day_Afternoon_Solar_Bucket[2]=Afternoon_Solar_Bucket;
        MA_Afternoon_Solar_Bucket=(Day_Afternoon_Solar_Bucket[0]+Day_Afternoon_Solar_Bucket[1]+Day_Afternoon_Solar_Bucket[2])/3;
        reset_solar_power_bucket();
}

void reset_solar_power_bucket(){
    for(int i=0;i<12;i++){
        solar_power_bucket[i]=0.0;
    }
    Morning_Solar_Bucket=0.0;
    Afternoon_Solar_Bucket=0.0;
}

void reset_load_power_bucket(){
    for(int i=0;i<24;i++){
        load_power_bucket[i]=0.0;
    }
    Day_Load_Bucket=0.0;
    Night_Load_Bucket=0.0;
}
void Load_Energy_Bucketing(){
        Day_Load_Bucket=0.0;
        for (int i = 0; i < Sundown_Time; i++){
			Day_Load_Bucket=Day_Load_Bucket+load_power_bucket[i];
		}

        Night_Load_Bucket=0.0;
		for (int i = Sundown_Time+6; i <= 23; i++){
			Night_Load_Bucket=Night_Load_Bucket+load_power_bucket[i];
		}
		Day_Morning_Load_Bucket[0]=Day_Morning_Load_Bucket[1];
        Day_Morning_Load_Bucket[1]=Day_Morning_Load_Bucket[2];
        Day_Morning_Load_Bucket[2]=Day_Morning_Load_Bucket[3];
        Day_Morning_Load_Bucket[3]=Day_Morning_Load_Bucket[4];
        Day_Morning_Load_Bucket[4]=Day_Morning_Load_Bucket[5];
        Day_Morning_Load_Bucket[5]=Day_Morning_Load_Bucket[6];
        Day_Morning_Load_Bucket[6]=Day_Load_Bucket;      
        Day_Night_Load_Bucket[0]=Day_Night_Load_Bucket[1];
        Day_Night_Load_Bucket[1]=Day_Night_Load_Bucket[2];
        Day_Night_Load_Bucket[2]=Day_Night_Load_Bucket[3];
        Day_Night_Load_Bucket[3]=Day_Night_Load_Bucket[4];
        Day_Night_Load_Bucket[4]=Day_Night_Load_Bucket[5];
        Day_Night_Load_Bucket[5]=Day_Night_Load_Bucket[6];
        Day_Night_Load_Bucket[6]=Night_Load_Bucket;
        reset_load_power_bucket();
}

void Dynamic_FT_Exit_Algorithm(){
    //Only if Dynamic_FT_Exit_Enable is set the algorithm works
    //For the first 4 days the dynamic algorithm doesn't work
    //When useable SOC is to be calculated the algorithm won't work
    if(Dynamic_FT_Exit_Enable){//Solar Bucketing Code
        if((Previous_Day_Morning_Sunhour<=Solar_Time)&&(Correction_Factor_Enable==0)){
            Correction_Factor=((MA_Afternoon_Solar_Bucket/MA_Morning_Solar_Bucket)*Today_Recieved_Solar_Energy)+Today_Recieved_Solar_Energy;
            Correction_Factor_Enable=1;
        }

        if(Correction_Factor_Enable){
            if((Correction_Factor-Today_Recieved_Solar_Energy-(SOC_Error*SOC_Err_Inc))<=((total_bat_capacity)-Bat_SOC)){
                Dynamic_FT_Exit=1;
                Correction_Factor_Enable=0;
            }
        }
        
        if(test_common_flag(CF_FT_EXIT)){
            if(total_time>server_ft_exit_time){
               Dynamic_FT_Exit=1; 
            }
        }
    }
}

uint32_t  
Dynamic_FT_Entry_Time_Calculate(){
    //This algorithm works only when the dynamic ft entry enable flag is set
    //The flag will be reset during absorption days
    if(Dynamic_FT_Entry_Enable){
        if(dev_state_flag&Bit(0)){
            //The morning load and evening load will be predicted
            //This section predicts considering the previous week same day load and average of the last week load.
            B1=0.75*Day_Morning_Load_Bucket[0]+0.25*((Day_Morning_Load_Bucket[1]+Day_Morning_Load_Bucket[2]+Day_Morning_Load_Bucket[3]+Day_Morning_Load_Bucket[4]+Day_Morning_Load_Bucket[5]+Day_Morning_Load_Bucket[6])/6);
            B2=0.75*Day_Night_Load_Bucket[0]+0.25*((Day_Night_Load_Bucket[1]+Day_Night_Load_Bucket[2]+Day_Night_Load_Bucket[3]+Day_Night_Load_Bucket[4]+Day_Night_Load_Bucket[5]+Day_Night_Load_Bucket[6])/6);
        }
        else{            
            //This sections predicts considering previous week same day load and yesterday's load
            B1=0.75*Day_Morning_Load_Bucket[0]+0.25*Day_Morning_Load_Bucket[6];
            B2=0.75*Day_Night_Load_Bucket[0]+0.25*Day_Night_Load_Bucket[6];
        }

        //Depth of discharge is calculated along with an AH efficiency        
        C2=(((total_bat_capacity)-Bat_SOC))/AH_Efficiency;

        //If the morning and afternoon bucket of next day is greater than the night load and depth of discharge estimated
        if((MA_Morning_Solar_Bucket+MA_Afternoon_Solar_Bucket)>(B1+C2)){
            //Calculate the FT constant which will be used in estimating the ft entry time
            if((MA_Morning_Solar_Bucket+MA_Afternoon_Solar_Bucket-B1-C2)>useable_SOC1){
                FT_Constant=(useable_SOC1/B2);
            }
            else{
                if((MA_Morning_Solar_Bucket+MA_Afternoon_Solar_Bucket-B1-C2)>B2){
                    FT_Constant=1.0;
                }
                else{
                    FT_Constant=(B2-(MA_Morning_Solar_Bucket+MA_Afternoon_Solar_Bucket-B1-C2))/B2;
                }
            }
            //Enable ready to frc trip flag
            ready_for_frc_trip=1;
            //Reset no force trip flag
            no_frc_trip_flag=0;
            //Enable ft entry
            Dynamic_FT_Enter=1;
            //FT entry time is calculated based on FT_constant

            if(FT_Constant>=1.0)
            {
                FT_Constant=1.0;
            }

            //FT_Entry_Time=(uint32_t)((24.0-(7.0*FT_Constant))*(float)3600);
            FT_Entry_Time=(uint32_t) ((((float)Sundown_Time+(float)6.0)+((float)6.0*(float)(1.0-FT_Constant)))*(float)3600);
        }
        else
        {
            Dynamic_FT_Enter=0;
#if 1
            if((Bat_SOC>0.95*(total_bat_capacity))&&(solar_equ_cnt>3600))
            {
                FT_Constant=(useable_SOC1/B2);
                //Enable ready to frc trip flag
                    ready_for_frc_trip=1;
                //Reset no force trip flag
                    no_frc_trip_flag=0;
                //Enable ft entry
                    Dynamic_FT_Enter=1;
                //FT entry time is calculated based on FT_constant

                if(FT_Constant>=1.0)
                {
                    FT_Constant=1.0;
                }

                //FT_Entry_Time=(uint32_t)((24.0-(7.0*FT_Constant))*(float)3600);
                FT_Entry_Time=(uint32_t) ((((float)Sundown_Time+(float)6.0)+((float)6.0*(float)(1.0-FT_Constant)))*(float)3600);
                
            }
            else
            {
                Dynamic_FT_Enter=0;
            }
#endif
        }
    }
        if(Dynamic_FT_Enter==0)
        {
            return 0;
        }
        else
        {
            return FT_Entry_Time;
        }
}

void 
Dynamic_FT_Entry_Algorithm()
{
    
    if((total_time>estimated_ft_entry_time)&&(Dynamic_FT_Enter==1))
    {
        check_ft_entry_time=Dynamic_FT_Entry_Time_Calculate();
                
        if((check_ft_entry_time-estimated_ft_entry_time)>600)
        {
            Dynamic_FT_Enter=1;
            estimated_ft_entry_time=check_ft_entry_time;
        }
        else
        {
            is_time_to_dynamic_ft_entry=1;
            ready_for_frc_trip=1;
            Dynamic_FT_Enter=0;
        }
    }
}

void 
sun_down_routine()
{
        sundown_count=0;
        solar_cycling=0;
        mains_sol_chg_flag=0;
        if(test_system_flag(SF_SMC))
        {
         clear_system_flag(SF_SMC);
        }
        mains_chg_flag=0;
        Sundown_Time=Solar_Time;
        
        
        Morning_Sunhour=Sundown_Time/2;
        
        if(num_days>1)
        {
            Previous_Day_Sundown=Sundown_Time;
            Previous_Day_Morning_Sunhour=Morning_Sunhour;
        }

        /*The variable indicates the day or night time of the day*/
            is_day_flag = NOW_NIGHT_TIME;
        /*Total solar received for the day is reset*/
            Today_Recieved_Solar_Energy=0;

        /*Solar value collected during the day is bucketed into morning and evening slots*/

        if((num_days>=2)&&!(dev_state_flag&Bit(5)))
        {
            Solar_Energy_Bucketing();
        }
        /*Variables assigned to stop the force trip are reset*/    
            no_frc_trip_flag = 0;
            frc_trip_count = 0;

        /*This condition happens during the first day of installation or when a useable SOC mismatch happens*/
        /*When the SF_USEABLE_SOC_CAL flag is set. The device is pushed into absorption. Equalization is done on the next day*/
        /*Once absorption and equalization is complete the battery is evacuated to estimate the useable SOC*/
            if((++useable_soc_recal_cnt>useable_soc_recal_dur)||(test_common_flag(CF_RPT_INIT)))
            {
                post_system_flag(SF_USEABLE_SOC_CAL);
                useable_soc_recal_cnt=0;
                
                if(test_common_flag(CF_RPT_INIT))
                {
                    clear_common_flag(CF_RPT_INIT);
                    i2c_eeprom_write_buff(0xFE7F,(uint8_t *) &common_flag,2);
                }
            }
            
            if(test_system_flag(SF_USEABLE_SOC_CAL)&&!(test_common_flag(CF_BYSOC)))
            {
                clear_system_flag(SF_USEABLE_SOC_CAL);
                enable_cal_useable_soc=1;
                post_system_flag(SF_ABS_NEEDED);
                
                if(battery_type==TUBULAR)
                {
                    need_eq_after_abs=1;
                }
                else
                {
                    need_eq_after_abs=0;
                }
            }
            else
            {
                if(test_common_flag(CF_BYSOC))
                {
                    clear_system_flag(SF_USEABLE_SOC_CAL);
                    post_system_flag(SF_ABS_NEEDED);
                    clear_common_flag(CF_BYSOC);
                    if(!(dev_state_flag&Bit(2)))
                    {
                        dev_state_flag|=Bit(2);
                    }
                    i2c_eeprom_write_buff(0xFE7F,(uint8_t *) &common_flag,2);
                }   
            }

            if(test_system_flag(SF_ABS_NEEDED)||(test_common_flag(CF_SET_ABS))||(test_common_flag(CF_SET_EQU)))
            {
                Dynamic_FT_Entry_Enable=0;
                clear_system_flag(SF_ABS_MF);
                clear_system_flag(SF_ABS_NEEDED);
                clear_system_flag(SF_ABS_COMPLETED);
                no_inh_flag=1; /* During Usable SoC calculation, inhibition should not happen*/
                if(test_common_flag(CF_SET_EQU))
                {
                    need_eq_after_abs=1;
                    clear_common_flag(CF_SET_EQU);
                    i2c_eeprom_write_buff(0xFE7F,(uint8_t *) &common_flag,2);
                }
        /*Since equalization doesn't happen after every absorption. This flag tells us whether to do an equalization or not after absorption*/
                if(need_eq_after_abs)
                {
                    is_time_to_equ=1;
                }
                else
                {
                    is_time_to_equ=0;
                }
        /*Every time the absorption is completed the battery soc has to be reset*/
        /*The soc is reset the first time battery voltage hits bat_max_v after absorption*/                
                soc_reset_flag=0;
                if(test_common_flag(CF_SET_ABS))
                {
                    clear_common_flag(CF_SET_ABS);
                    i2c_eeprom_write_buff(0xFE7F,(uint8_t *) &common_flag,2);
                }                
            }

            Dynamic_FT_Exit=0x00;
            dy_exit=0;
            Dynamic_FT_Exit_Enable = 0;
            
        /*Once dynamic force trip entry is enabled, at sundown the time at which the entry should happen is estimated*/
        /*On the time of entry the time is again calculated and checked with this value inorder to ensure that battery did not discharge between that time period*/
            estimated_ft_entry_time=Dynamic_FT_Entry_Time_Calculate();
            
            if(((algo_param.dev_algo_state == ALGO_STATE_BAT_SOL_FRC_TRIP_6) || (algo_param.dev_algo_state == ALGO_STATE_BAT_FRC_TRIP_4))&&(is_time_to_cal_useable_soc==0))
            {
                algo_param.dev_algo_state = ALGO_STATE_INV_5;
                turtle_set_sw(algo_param.dev_algo_state);
            }
}

void show_led(){
	uint16_t led_colour;
    if((test_error(ANY_ERROR)) || (pac_error_flag)){ //If any error or pac error	
        if(test_error(ERR_NO_MEMORY)){
            ESP_LOGI(TAG,"EEPROM memory allocation");
            blink(led_red_color,FULL_ON);
        }
        else if(test_error((ERR_NOT_CALIB))){
            ESP_LOGI(TAG,"Not calibrated");
            blink(led_red_color,ONE_SECOND_BLINK);
        }
        else if(test_error((ERR_NOT_CONFIG))){
            ESP_LOGI(TAG,"Not configured");
            blink(led_magenta_color,ONE_SECOND_BLINK);
        }
        else if(pac_error_flag){
            blink(led_red_color,FULL_ON);
        }
    }
    else{
        if((stop_algorithm)||(device_state==ALGO_OFF)){
            if(device_state==ALGO_OFF){
                blink(led_no_color,ONE_SECOND_BLINK);
            }
        }
        else
        {
            if(is_sol_sensed == 0){
                ESP_LOGI(TAG,"Solar not detected");
                blink(led_magenta_color,FIVE_SECOND_BLINK);
            }
            else{
                if(test_system_flag(SF_INV_FAIL)){
                    ESP_LOGI(TAG,"Inverter failure");
                    blink(led_red_color,ONE_SECOND_BLINK);
                }
                else{	           
                    if(battery_low==1){
                        ESP_LOGI(TAG,"Battery low");
                        blink(led_red_color,ONE_SECOND_BLINK);
                    }
                    else{
                        if(mains_available==0){
                        // if(0){
                            ESP_LOGI(TAG,"Mains failure");
                            blink(led_red_color,FIVE_SECOND_BLINK);
                        }
                        else{
                            switch(algo_param.dev_algo_state){
                                case ALGO_STATE_INV_5:  
                                    ESP_LOGI(TAG,"State 5");
                                    blink(led_blue_color,ONE_SECOND_BLINK);
                                    break;
                                case ALGO_STATE_CHARG_INHIBIT_1: 
                                    ESP_LOGI(TAG,"State 1");
                                    blink(led_blue_color,FIVE_SECOND_BLINK);
                                    break;
                                case ALGO_STATE_SOL_CHG_3: 
                                    ESP_LOGI(TAG,"State 3");
                                    blink(led_blue_color,FIVE_SECOND_BLINK);
                                    break;	
                                case ALGO_STATE_BAT_FRC_TRIP_4: 
                                    ESP_LOGI(TAG,"State 4");
                                    blink(led_green_color,FIVE_SECOND_BLINK);
                                    break;	
                                case ALGO_STATE_BAT_SOL_FRC_TRIP_6: 
                                    ESP_LOGI(TAG,"State 6");
                                    blink(led_green_color,FIVE_SECOND_BLINK);
                                    break; 
                                case ALGO_STATE_SOL_INV_DIS_7: 
                                    ESP_LOGI(TAG,"State 7");
                                    blink(led_green_color,ONE_SECOND_BLINK);
                                    break;		
                            }
                        }
                    }
                }	
            }
        }
    }
}