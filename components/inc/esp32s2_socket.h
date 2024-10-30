#ifndef _esp32s2_socket_
#define _esp32s2_socket_

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"
#include "esp32s2_algorithm.h"
#include "esp32s2_gpio.h"

#define CONFIG_CALIB_WIFI_SERVER	1
#define CONFIG_CALIB_WIFI_CLIENT	2
#define CONFIG_CALIB_UART			3 

#define Bit(x) (1UL << (x))

typedef enum{
    REQ_CON 			= 7,
    CMD_GET_ADC 		= 1,
    CMD_SET_TIME		= 2,
    CMD_GET_SW 			= 3,
    CMD_SET_SW 			= 4,
    CMD_WRITE_EEPROM	= 5,
    CMD_READ_EEPROM		= 6,
    CMD_GET_REAL_VAL 	= 8,
    CMD_TEST_CONFIG		= 9,
    CMD_EXIT 			= 0x0A,
    CMD_BUZZER			= 0x0B,
    CMD_SERV_BEACON		= 11,
    CMD_GET_ADC_MV_AVG	= 12,
    CMD_GET_CAL_CONST	= 13,
}req_commands;


typedef enum{
    CONFIG_CALIB_SYNC	        = 0xCC, 
    MAX_PAYLOAD_LEN		        = 128,		
    REQ_ID_IDX			        = 0,
    REQ_SES_ID_IDX 		        = 1,
    REQ_DAT_LEN_IDX		        = 5,
    REQ_PAYLOAD_IDX 	        = 7, 
    SES_ID_LEN			        = 4,
    REQ_CRC_LEN			        = 2,
    REQ_HEADER_LEN		        = 7,
    RES_SYNC_IDX		        = 0,
    RES_ID_IDX 			        = 1,
    RES_DAT_LEN_IDX             = 2, 
    RES_SSID_IDX		        = 4,
    RES_PAYLOAD_IDX_CLIENT      = 8,
    RES_PAYLOAD_IDX		        = 4, 
    RES_HEADER_LEN		        = 4,
    RES_HEADER_LEN_CLIENT       = 8,
    RES_CRC_LEN			        = 2, 
    CON_RES_FLAG_IDX	        = 0,
    CON_RES_ID_IDX		        = 1,
    CON_RES_HW_VER_IDX	        = 14,
    CON_RES_FW_VER_IDX	        = 16 , 
    CON_RES_PROT_VER_IDX	    = 18,
    CON_RES_SID_IDX			    = 20,
    CON_RES_PKT_LEN			    = 24, 
}packet_indices;

#define CALIB_TIMEOUT 		300000

typedef enum{
    EE_CONFIG_DAT_START_ADDR 	    = 0xFC00,
    EE_CONFIG_DAT_END_ADDR 		    = 0xFCFF, 
    EE_SERV_IP_ADDR 		        = EE_CONFIG_DAT_START_ADDR,
    SERV_IP_LEN				        = 32,
    EE_SERV_PORT_ADDR 		        = EE_SERV_IP_ADDR + SERV_IP_LEN,
    SERV_PORT_LEN 			        = 2,
    EE_AP_SSID_ADDR			        = EE_SERV_PORT_ADDR + SERV_PORT_LEN,
    AP_SSID_LEN 			        = 32,
    EE_AP_PSW_ADDR 			        = EE_AP_SSID_ADDR + AP_SSID_LEN,
    AP_PSW_LEN 				        = 32,
    EE_DC_OVER_I_THR_ADDR	        = EE_AP_PSW_ADDR + AP_PSW_LEN,
    DC_OVER_I_THR_LEN		        = 4,
    EE_AC_OVER_I_THR_ADDR 	        = EE_DC_OVER_I_THR_ADDR + DC_OVER_I_THR_LEN,
    AC_OVER_I_THR_LEN 		        = 4,
    EE_BAT_TYPE_ADDR                = EE_AC_OVER_I_THR_ADDR+AC_OVER_I_THR_LEN,
    EE_BAT_TYPE_LEN                 = 1,
    EE_BAT_MAX_VOLT_ADDR 	        = EE_BAT_TYPE_ADDR + EE_BAT_TYPE_LEN,
    BAT_MAX_VOLT_LEN 		        = 4,
    EE_BAT_CAP_ADDR                 = EE_BAT_MAX_VOLT_ADDR+BAT_MAX_VOLT_LEN,
    EE_BAT_CAP_LEN                  = 4,
    EE_BAT_AGE_ADDR                 = EE_BAT_CAP_ADDR+EE_BAT_CAP_LEN,
    EE_BAT_AGE_LEN                  = 1,
    EE_BAT_FRC_EXIT_VOLT_ADDR 	    = EE_BAT_AGE_ADDR + EE_BAT_AGE_LEN,
    BAT_FRC_EXIT_VOLT 			    = 4,
    EE_FT_NUM_ADDR                  = EE_BAT_FRC_EXIT_VOLT_ADDR+BAT_FRC_EXIT_VOLT,
    EE_FT_NUM_LEN                   = 1,
    EE_SOL_CAP                      = EE_FT_NUM_ADDR+EE_FT_NUM_LEN,
    EE_SOL_CAP_LEN                  = 4,
    EE_BAT_MAINS_CHG_IN_VOLT_ADDR	= EE_SOL_CAP + EE_SOL_CAP_LEN,
    BAT_MAINS_CHG_IN_VOLT_LEN		= 4,
    EE_BAT_MAINS_CHG_OUT_VOLT_ADDR	= EE_BAT_MAINS_CHG_IN_VOLT_ADDR + BAT_MAINS_CHG_IN_VOLT_LEN,
    BAT_MAINS_CHG_OUT_VOLT_LEN		= 4,
    EE_MORN_CHG_INH_EN_ADDR	        = EE_BAT_MAINS_CHG_OUT_VOLT_ADDR + BAT_MAINS_CHG_OUT_VOLT_LEN,
    MORN_CHG_INH_EN_LEN		        = 1,
    EE_EVEN_CHG_INH_EN_ADDR	        = EE_MORN_CHG_INH_EN_ADDR + MORN_CHG_INH_EN_LEN,
    EVEN_CHG_INH_EN_LEN		        = 1,
    EE_ABS_INTR_ADDR                = EE_EVEN_CHG_INH_EN_ADDR+EVEN_CHG_INH_EN_LEN,
    EE_ABS_INTR_LEN                 = 1,
    EE_BAT_EQU_INTR_ADDR	        = EE_ABS_INTR_ADDR + EE_ABS_INTR_LEN,
    BAT_EQU_INTR_LEN		        = 1,
    EE_BAT_EQU_DUR_ADDR		        = EE_BAT_EQU_INTR_ADDR + BAT_EQU_INTR_LEN,
    BAT_EQU_DUR_LEN			        = 1,
    EE_CONFIG_DONE_ADDR		        = EE_BAT_EQU_DUR_ADDR + BAT_EQU_DUR_LEN,
    CONFIG_DONE_LEN			        = 4,
    EE_CONFIG_CRC_ADDR		        = (EE_CONFIG_DONE_ADDR + CONFIG_DONE_LEN),
    CONFIG_CRC_LEN			        = 2, 
    CONFIG_PARAM_LEN		        = ( SERV_IP_LEN + SERV_PORT_LEN + AP_SSID_LEN + \
								        AP_PSW_LEN + DC_OVER_I_THR_LEN + AC_OVER_I_THR_LEN + \
                                        EE_BAT_TYPE_LEN + BAT_MAX_VOLT_LEN + EE_BAT_CAP_LEN + \
                                        EE_BAT_AGE_LEN + BAT_FRC_EXIT_VOLT + EE_FT_NUM_LEN + \
                                        EE_SOL_CAP_LEN + BAT_MAINS_CHG_IN_VOLT_LEN + \
								        BAT_MAINS_CHG_OUT_VOLT_LEN + MORN_CHG_INH_EN_LEN + EVEN_CHG_INH_EN_LEN + EE_ABS_INTR_LEN + \
								        BAT_EQU_INTR_LEN + BAT_EQU_DUR_LEN ),
}configuration_parameters_t;

typedef enum{
    EE_CALIB_DAT_START_ADDR     = 0xFD00,
    EE_CALIB_DAT_END_ADDR 	    = 0xFDFF,
    EE_DEV_SER_NO_ADDR 			= EE_CALIB_DAT_START_ADDR,
    DEV_SER_NO_LEN 				= 12, 
    EE_DEVICE_TYPE_ADDR			= EE_DEV_SER_NO_ADDR + DEV_SER_NO_LEN,
    DEVICE_TYPE_LEN				= 1, 
    EE_UC_BAT_V_CONST_ADDR 		= EE_DEVICE_TYPE_ADDR + DEVICE_TYPE_LEN,
    UC_BAT_V_CONST_LEN 			= 4,
    EE_PAC_BAT_V_CONST_ADDR 	= EE_UC_BAT_V_CONST_ADDR + UC_BAT_V_CONST_LEN,
    PAC_BAT_V_CONST_LEN 		= 4,
    EE_PAC_BAT_CHG_I_CONST_ADDR = EE_PAC_BAT_V_CONST_ADDR + PAC_BAT_V_CONST_LEN,
    PAC_BAT_CHG_I_CONST_LEN 	= 4,
    EE_PAC_BAT_DIS_I_CONST_ADDR = EE_PAC_BAT_CHG_I_CONST_ADDR + PAC_BAT_CHG_I_CONST_LEN,
    PAC_BAT_DIS_I_CONST_LEN 	= 4,
    EE_PAC_BAT_CHG_I_OFF_ADDR   = EE_PAC_BAT_DIS_I_CONST_ADDR + PAC_BAT_DIS_I_CONST_LEN,
    PAC_BAT_CHG_I_OFF_LEN       = 4,
    EE_PAC_BAT_DIS_I_OFF_ADDR   = EE_PAC_BAT_CHG_I_OFF_ADDR + PAC_BAT_CHG_I_OFF_LEN,
    PAC_BAT_DIS_I_OFF_LEN       = 4,
    EE_PAC_BAT_P_CONST_ADDR 	= EE_PAC_BAT_DIS_I_OFF_ADDR + PAC_BAT_DIS_I_OFF_LEN,
    PAC_BAT_P_CONST_LEN 		= 4,
    EE_UC_SOL_V_CONST_ADDR 		= EE_PAC_BAT_P_CONST_ADDR + PAC_BAT_P_CONST_LEN,
    UC_SOL_V_CONST_LEN 			= 4,
    EE_PAC_SOL_V_CONST_ADDR 	= EE_UC_SOL_V_CONST_ADDR + UC_SOL_V_CONST_LEN,
    PAC_SOL_V_CONST_LEN 		= 4,
    EE_PAC_SOL_I_CONST_ADDR 	= EE_PAC_SOL_V_CONST_ADDR + PAC_SOL_V_CONST_LEN,
    PAC_SOL_I_CONST_LEN 		= 4,
    EE_PAC_SOL_I_OFF_ADDR       = EE_PAC_SOL_I_CONST_ADDR + PAC_SOL_I_CONST_LEN,
    PAC_SOL_I_OFF_LEN           = 4,
    EE_PAC_SOL_P_CONST_ADDR 	= EE_PAC_SOL_I_OFF_ADDR + PAC_SOL_I_OFF_LEN,
    PAC_SOL_P_CONST_LEN 		= 4,
    EE_UC_TEMP1_CONST_ADDR      = EE_PAC_SOL_P_CONST_ADDR+PAC_SOL_P_CONST_LEN,
    UC_TEMP1_CONST_LEN          = 4,
    EE_UC_TEMP2_CONST_ADDR      = EE_UC_TEMP1_CONST_ADDR + UC_TEMP1_CONST_LEN,
    UC_TEMP2_CONST_LEN          = 4,
    EE_CT_LOAD_I_OFST_ADDR		= EE_UC_TEMP2_CONST_ADDR + UC_TEMP2_CONST_LEN,
    CT_LOAD_I_OFST_LEN			= 2,
    EE_CT_LOAD_I_CONST_ADDR		= EE_CT_LOAD_I_OFST_ADDR + CT_LOAD_I_OFST_LEN,
    CT_LOAD_I_CONST_LEN			= 4,
    EE_CALIB_DONE_ADDR			= EE_CT_LOAD_I_CONST_ADDR + CT_LOAD_I_CONST_LEN,
    CALIB_DONE_LEN				= 4,
    EE_CALIB_CRC_ADDR			= EE_CALIB_DONE_ADDR + CALIB_DONE_LEN,
    CALIB_CRC_LEN				= 2,
    CALIB_CONST_LEN				= ( DEV_SER_NO_LEN + DEVICE_TYPE_LEN + UC_BAT_V_CONST_LEN + \
									PAC_BAT_V_CONST_LEN + PAC_BAT_CHG_I_CONST_LEN + PAC_BAT_DIS_I_CONST_LEN+ PAC_BAT_CHG_I_OFF_LEN + PAC_BAT_P_CONST_LEN + \
									PAC_BAT_DIS_I_OFF_LEN + UC_SOL_V_CONST_LEN + PAC_SOL_V_CONST_LEN + PAC_SOL_I_CONST_LEN + \
									PAC_SOL_I_OFF_LEN + PAC_SOL_P_CONST_LEN + CT_LOAD_I_OFST_LEN + CT_LOAD_I_CONST_LEN),
}calibration_parameters_t;

typedef enum{
    EE_SYS_THR_START_ADDR               = 0xFE00,
    EE_SYS_THR_END_ADDR                 = 0xFF90,  
    DEV_USEABLE_SOC_MULTIPLIER          = EE_SYS_THR_START_ADDR,
    DEV_USEABLE_SOC_MULTIPLIER_LEN      = 4,
    DEV_EQU_HYS                         = DEV_USEABLE_SOC_MULTIPLIER+DEV_USEABLE_SOC_MULTIPLIER_LEN,
    DEV_EQU_HYS_LEN                     = 1,
    DEV_ABS_MF_PER                      = DEV_EQU_HYS+DEV_EQU_HYS_LEN,
    DEV_ABS_MF_PER_LEN                  = 1,
    EQ_MF_PER                           = DEV_ABS_MF_PER+DEV_ABS_MF_PER_LEN,
    EQ_MF_PER_LEN                       = 1,
    VFT_EXIT_HIGH                       = EQ_MF_PER+EQ_MF_PER_LEN,
    VFT_EXIT_HIGH_LEN                   = 4,
    VFT_EXIT_LOW                        = VFT_EXIT_HIGH+VFT_EXIT_HIGH_LEN,
    VFT_EXIT_LOW_LEN                    = 4,
    DEV_SYS_FLAG                        = VFT_EXIT_LOW+VFT_EXIT_LOW_LEN,
    DEV_SYS_FLAG_LEN                    = 1,
    DAILY_LOAD_THR                      = DEV_SYS_FLAG+DEV_SYS_FLAG_LEN,
    DAILY_LOAD_THR_LEN                  = 4,
    WEEKLY_LOAD_THR                     = DAILY_LOAD_THR+DAILY_LOAD_THR_LEN,
    WEEKLY_LOAD_THR_LEN                 = 4,
    NUM_DAYS                            = WEEKLY_LOAD_THR+WEEKLY_LOAD_THR_LEN,
    NUM_DAYS_LEN                        = 2,
    USEABLE_SOC                         = NUM_DAYS+NUM_DAYS_LEN,
    USEABLE_SOC_LEN                     = 4,
    RES_ENE_MUL                         = USEABLE_SOC+USEABLE_SOC_LEN,
    RES_ENE_MUL_LEN                     = 1, 
    VFT_EXT_THR                         = RES_ENE_MUL+RES_ENE_MUL_LEN,
    VFT_EXT_THR_LEN                     = 4,
    EVE_BAT_SOC_COR_LOW_LIM             = VFT_EXT_THR+VFT_EXT_THR_LEN,
    EVE_BAT_SOC_COR_LOW_LIM_LEN         = 4,
    EVE_BAT_SOC_COR_HIGH_LIM            = EVE_BAT_SOC_COR_LOW_LIM+EVE_BAT_SOC_COR_LOW_LIM_LEN,
    EVE_BAT_SOC_COR_HIGH_LIM_LEN        = 4,
    SOL_MAX_CUR                         = EVE_BAT_SOC_COR_HIGH_LIM+EVE_BAT_SOC_COR_HIGH_LIM_LEN,
    SOL_MAX_CUR_LEN                     = 4,
    USEABLE_SOC_STRT                    = SOL_MAX_CUR+SOL_MAX_CUR_LEN,
    USEABLE_SOC_STRT_LEN                = 1,
    FT_ENTRY_STRT_DAY                   = USEABLE_SOC_STRT+USEABLE_SOC_STRT_LEN,
    FT_ENTRY_STRT_DAY_LEN               = 1  ,
    FT_EXIT_STRT_DAY                    = FT_ENTRY_STRT_DAY+FT_ENTRY_STRT_DAY_LEN,
    FT_EXIT_STRT_DAY_LEN                = 1 ,
    STATE_CHANGE_LOG_UNITS              = FT_EXIT_STRT_DAY+FT_EXIT_STRT_DAY_LEN,
    STATE_CHANGE_LOG_UNITS_LEN          = 1,
    STATE_CHANGE_LOG_UNITS_NO_WIFI      = STATE_CHANGE_LOG_UNITS+STATE_CHANGE_LOG_UNITS_LEN,
    STATE_CHANGE_LOG_UNITS_NO_WIFI_LEN  = 1,
    SOL_E_CORR                          = STATE_CHANGE_LOG_UNITS_NO_WIFI+STATE_CHANGE_LOG_UNITS_NO_WIFI_LEN,
    SOL_E_CORR_LEN                      = 4,
    SER_ASSERT_STATE                    = SOL_E_CORR+SOL_E_CORR_LEN,
    SER_ASSERT_STATE_LEN                = 1,
    SER_STATE_ASSERTED_TIM_HR           = SER_ASSERT_STATE+SER_ASSERT_STATE_LEN,
    SER_STATE_ASSERTED_TIM_HR_LEN       = 1,
    SER_STATE_ASSERTED_TIM_MIN          = SER_STATE_ASSERTED_TIM_HR+SER_STATE_ASSERTED_TIM_HR_LEN,
    SER_STATE_ASSERTED_TIM_MIN_LEN      = 1,
    SER_STATE_ASSERTED_DUR              = SER_STATE_ASSERTED_TIM_MIN+SER_STATE_ASSERTED_TIM_MIN_LEN,
    SER_STATE_ASSERTED_DUR_LEN          =  2,
    FT_EXIT_TIM                         = SER_STATE_ASSERTED_DUR+SER_STATE_ASSERTED_DUR_LEN,
    FT_EXIT_TIM_LEN                     = 2,
    MORN_SOL_PRED                       = FT_EXIT_TIM+FT_EXIT_TIM_LEN,
    MORN_SOL_PRED_LEN                   = 4,
    EVEN_SOL_PRED                       = MORN_SOL_PRED+MORN_SOL_PRED_LEN,
    EVEN_SOL_PRED_LEN                   = 4,
    DAY1_MORN_LOAD                      = EVEN_SOL_PRED+EVEN_SOL_PRED_LEN,
    DAY1_MORN_LOAD_LEN                  = 4,
    DAY1_NGT_LOAD                       = DAY1_MORN_LOAD+DAY1_MORN_LOAD_LEN,
    DAY1_NGT_LOAD_LEN                   = 4,
    DAY2_MORN_LOAD                      = DAY1_NGT_LOAD+DAY1_NGT_LOAD_LEN,
    DAY2_MORN_LOAD_LEN                  = 4,
    DAY2_NGT_LOAD                       = DAY2_MORN_LOAD+DAY2_MORN_LOAD_LEN,
    DAY2_NGT_LOAD_LEN                   = 4,
    DAY3_MORN_LOAD                      = DAY2_NGT_LOAD+DAY2_NGT_LOAD_LEN,
    DAY3_MORN_LOAD_LEN                  = 4,
    DAY3_NGT_LOAD                       = DAY3_MORN_LOAD+DAY3_MORN_LOAD_LEN,
    DAY3_NGT_LOAD_LEN                   = 4,
    DAY4_MORN_LOAD                      = DAY3_NGT_LOAD+DAY3_NGT_LOAD_LEN,
    DAY4_MORN_LOAD_LEN                  = 4,
    DAY4_NGT_LOAD                       = DAY4_MORN_LOAD+DAY4_MORN_LOAD_LEN,
    DAY4_NGT_LOAD_LEN                   = 4,
    DAY5_MORN_LOAD                      = DAY4_NGT_LOAD+DAY4_NGT_LOAD_LEN,
    DAY5_MORN_LOAD_LEN                  = 4,
    DAY5_NGT_LOAD                       = DAY5_MORN_LOAD+DAY5_MORN_LOAD_LEN,
    DAY5_NGT_LOAD_LEN                   = 4,
    DAY6_MORN_LOAD                      = DAY5_NGT_LOAD+DAY5_NGT_LOAD_LEN,
    DAY6_MORN_LOAD_LEN                  = 4,
    DAY6_NGT_LOAD                       = DAY6_MORN_LOAD+DAY6_MORN_LOAD_LEN,
    DAY6_NGT_LOAD_LEN                   = 4,
    DAY7_MORN_LOAD                      = DAY6_NGT_LOAD+DAY6_NGT_LOAD_LEN,
    DAY7_MORN_LOAD_LEN                  = 4,
    DAY7_NGT_LOAD                       = DAY7_MORN_LOAD+DAY7_MORN_LOAD_LEN,
    DAY7_NGT_LOAD_LEN                   = 4,
    COMMON_THR                          = 0xFE7F,
    COMMON_THR_LEN                      = 2, 
    DEV_SOC_VOL_TABLE                   = COMMON_THR+COMMON_THR_LEN,
    DEV_SOC_VOL_TABLE_LEN               = 245,
    DAY1_MORN_SOL                       = 0xFF76,
    DAY1_MORN_SOL_LEN                   = 4,
    DAY1_AFT_SOL                        = DAY1_MORN_SOL+DAY1_MORN_SOL_LEN,
    DAY1_AFT_SOL_LEN                    = 4,
    DAY2_MORN_SOL                       = DAY1_AFT_SOL+DAY1_AFT_SOL_LEN,
    DAY2_MORN_SOL_LEN                   = 4,
    DAY2_AFT_SOL                        = DAY2_MORN_SOL+DAY2_MORN_SOL_LEN,
    DAY2_AFT_SOL_LEN                    = 4,
    DAY3_MORN_SOL                       = DAY2_AFT_SOL+DAY2_AFT_SOL_LEN,
    DAY3_MORN_SOL_LEN                   = 4,
    DAY3_AFT_SOL                        = DAY3_MORN_SOL+DAY3_MORN_SOL_LEN,
    DAY3_AFT_SOL_LEN                    = 4,
}system_threshold_parameters_t;

typedef enum{
    TURT_STAT_OFF = 0,
    TURT_STAT_ON  = 1,
    ALGO_OFF      = 0,  
    ALGO_ON       = 1,
}algorithm_status_t;

typedef enum{
    SF_SOL_DIS            = Bit(0),  //Configuration incremented flag
    SF_SMC                = Bit(1),  //Solar+Mains Charging flag
    EVE_SOC_ERR           = Bit(2),  //Threshold updated by device flag
    NGT_SOC_ERR           = Bit(3),  //Threshold updated by server flag
    SF_ABS_NEEDED         = Bit(4),  //Absorption Needed flag
    SF_ABS_COMPLETED      = Bit(5),  //Absorption Completed flag
    SF_ABS_MF             = Bit(6),  //Mains fail absorption flag
    SF_EQ_NEEDED          = Bit(7),  //Equalization needed flag
    SF_EQ_COMPLETED       = Bit(8),  //Equalization completed flag
    SF_EQ_FAILED          = Bit(9),  //Equalization failed flag
    SF_INIT_COMPLETED     = Bit(10), //Initialization completed flag
    SF_INIT_FAILED        = Bit(11), //Initialization failure flag
    SF_INV_FAIL           = Bit(12), //FT Voltage exit threshold set flag
    SF_USEABLE_SOC_RC     = Bit(13), //Useable SOC Rule change flag
    SF_USEABLE_SOC_FAILED = Bit(14), //Useable SOC Error Flag
    SF_USEABLE_SOC_CAL    = Bit(15), //Enable Useable SOC Calculation
}system_flag_t;

typedef enum{
    CF_CLR_ALL    = 0xFFFF, //Clear all flags
    CF_SS         = Bit(0),//Server state assert flag
    CF_FT_EXIT    = Bit(1),//FT exit time set flag 
    CF_BYSOC      = Bit(2),//Bypass useable SoC calculation flag
    CF_RPT_INIT   = Bit(3),//Repeat initialization flag 
    CF_SOC_ERR    = Bit(4),//SoC error include flag
    CF_BAT_TEMP   = Bit(5),//Battery temperature compensation
    CF_SOL_CR     = Bit(6),//Solar energy correction
    CF_SOL_E_UPD  = Bit(7),//Server Sol E update
    CF_LOAD_E_UPD = Bit(8),//Server Load E update
    CF_RESET_DEV  = Bit(9),//Reset device from server
    CF_DAY        = Bit(10),//Change Date
    CF_SET_ABS    = Bit(11),//Set absorption
    CF_SET_EQU    = Bit(12),//Set equalization
}common_flags_t;

typedef enum{
    HW_VERSION = 0x0300,
    FW_VERSION = 0x0810,
}version_t;

extern volatile uint16_t system_flag;
extern volatile uint16_t common_flag;
extern volatile uint8_t server_flag;
extern volatile uint8_t error_flag;

#define post_system_flag(x)  (system_flag |= (x))
#define test_system_flag(x)  (system_flag & (x))
#define clear_system_flag(x) (system_flag &= ~(x))

#define post_common_flag(x)  (common_flag |= (x))
#define test_common_flag(x)  (common_flag & (x))
#define clear_common_flag(x) (common_flag &= ~(x))

#define ERR_NO_MEMORY		Bit(0) 
#define ERR_NOT_CALIB		Bit(1) 
#define ERR_NOT_CONFIG		Bit(2) 
#define ERR_RTC 			Bit(3) 
#define ERR_RTC_SYNC		Bit(4)
#define ANY_ERROR			(ERR_NO_MEMORY | ERR_NOT_CALIB | \
							ERR_NOT_CONFIG | ERR_RTC_SYNC)


#define post_error(x)		(error_flag |= (x))
#define test_error(x)		(error_flag & (x))
#define clear_error(x)		(error_flag &= ~(x)) 


#define FLASH_FM_VER_ADDR		0xFBC0
#define CALIB_CONFIG_DONE_VALUE		0x22334466
#define HW_FM_VER_LEN			4
#define CALIB_CONFIG_MAJ_VER	0x01
#define CALIB_CONFIG_MIN_VER	0x00
#define CALIB_CONFIG_VER_LEN	2
#define CALIB_CONFIG_AUTH_FUTURE_BYTES_LEN	10

#define AC_POWER_CONST	(220.0 * 0.8)

#define UART_STREAM_RX_BUFF_SIZE	150
#define UART_STREAM_TX_BUFF_SIZE	150

typedef enum{
    idle,
    allocated_socket,
    connected_socket,
    bind_socket,
    listen_socket,
    new_connection,
    data_received,
    do_nothing,
}socket_client_status_t;


#define RECV_BUFF_SIZE 128
extern uint8_t recv_buf[RECV_BUFF_SIZE], recv_buf_head, recv_buf_tail;

#define SWAPNIBBLE(x) (((x) << 4) | ((x) >> 4))
#define SWAP_KEY1		0x55
#define SWAP_KEY2		0xAA
#define POLY 0x8408

void close_socket(void);
void deleteConfigCalibTask(void);
void tcp_client(void *pvParam);
void socket_data_parse(uint8_t * _data);
void tcp_server(void *pvParam);
uint8_t read_from_rf(uint8_t *read_byte);
uint16_t crc_16(uint8_t *data_p, uint16_t length);
uint8_t read_calib_config_req(uint8_t *req_cmd, uint8_t *req_payload, uint16_t *req_pkt_len);
void res_to_utility(uint8_t _res_type, uint8_t *_res_buff, uint16_t _res_len);
void send_con_res_config_calib();
void process_labview_cmd();
uint16_t calc_update_eeprom_crc(uint16_t _ee_start_address, uint8_t _ee_len, uint16_t _crc_addr, uint8_t is_update);

#endif