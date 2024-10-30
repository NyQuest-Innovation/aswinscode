#include "esp32s2_parse_data.h"
#include "esp32s2_socket.h"
#include "esp32s2_24lc512.h"
#include "esp32s2_main.h"

#define TAG "parseJSON"

extern https_request_buffer_body_t https_request_buffer_body;

uint16_t write_info_address[98]={
  EE_SERV_IP_ADDR, EE_SERV_PORT_ADDR, EE_AP_SSID_ADDR, EE_AP_PSW_ADDR,    
  EE_DC_OVER_I_THR_ADDR, EE_AC_OVER_I_THR_ADDR,
  EE_BAT_TYPE_ADDR, EE_BAT_MAX_VOLT_ADDR,               		        
  EE_BAT_CAP_ADDR, EE_BAT_AGE_ADDR, 
  EE_BAT_FRC_EXIT_VOLT_ADDR, EE_FT_NUM_ADDR,    
  EE_SOL_CAP,                               
  EE_BAT_MAINS_CHG_IN_VOLT_ADDR, EE_BAT_MAINS_CHG_OUT_VOLT_ADDR,
  EE_MORN_CHG_INH_EN_ADDR, EE_EVEN_CHG_INH_EN_ADDR, EE_ABS_INTR_ADDR, EE_BAT_EQU_INTR_ADDR, EE_BAT_EQU_DUR_ADDR,
  EE_DEV_SER_NO_ADDR, EE_DEVICE_TYPE_ADDR,    
  EE_UC_BAT_V_CONST_ADDR, EE_PAC_BAT_V_CONST_ADDR, 
  EE_PAC_BAT_CHG_I_CONST_ADDR, EE_PAC_BAT_DIS_I_CONST_ADDR, 
  EE_PAC_BAT_CHG_I_OFF_ADDR, EE_PAC_BAT_DIS_I_OFF_ADDR,
  EE_PAC_BAT_P_CONST_ADDR, EE_UC_SOL_V_CONST_ADDR, 
  EE_PAC_SOL_V_CONST_ADDR, EE_PAC_SOL_I_CONST_ADDR, EE_PAC_SOL_I_OFF_ADDR, EE_PAC_SOL_P_CONST_ADDR,
  EE_UC_TEMP1_CONST_ADDR, EE_UC_TEMP2_CONST_ADDR, EE_CT_LOAD_I_OFST_ADDR, EE_CT_LOAD_I_CONST_ADDR,
  DEV_USEABLE_SOC_MULTIPLIER, DEV_EQU_HYS, 
  DEV_ABS_MF_PER, EQ_MF_PER, 
  VFT_EXIT_HIGH, VFT_EXIT_LOW,  
  DEV_SYS_FLAG, DAILY_LOAD_THR,   
  WEEKLY_LOAD_THR, NUM_DAYS,     
  USEABLE_SOC, RES_ENE_MUL, VFT_EXT_THR,            
  EVE_BAT_SOC_COR_LOW_LIM, EVE_BAT_SOC_COR_HIGH_LIM,                   
  SOL_MAX_CUR, USEABLE_SOC_STRT,                    
  FT_ENTRY_STRT_DAY, FT_EXIT_STRT_DAY,                 
  STATE_CHANGE_LOG_UNITS, STATE_CHANGE_LOG_UNITS_NO_WIFI,           
  SOL_E_CORR, SER_ASSERT_STATE, SER_STATE_ASSERTED_TIM_HR, SER_STATE_ASSERTED_TIM_MIN, 
  SER_STATE_ASSERTED_DUR, FT_EXIT_TIM, MORN_SOL_PRED, EVEN_SOL_PRED,
  DAY1_MORN_LOAD, DAY1_NGT_LOAD, 
  DAY2_MORN_LOAD, DAY2_NGT_LOAD,
  DAY3_MORN_LOAD, DAY3_NGT_LOAD, 
  DAY4_MORN_LOAD, DAY4_NGT_LOAD, 
  DAY5_MORN_LOAD, DAY5_NGT_LOAD,                       
  DAY6_MORN_LOAD, DAY6_NGT_LOAD, 
  DAY7_MORN_LOAD, DAY7_NGT_LOAD,
  COMMON_THR, DEV_SOC_VOL_TABLE,
  DAY1_MORN_SOL, DAY1_AFT_SOL,
  DAY2_MORN_SOL, DAY2_AFT_SOL,
  DAY3_MORN_SOL, DAY3_AFT_SOL
};

uint8_t write_info_length[98]={
  SERV_IP_LEN, SERV_PORT_LEN, AP_SSID_LEN, AP_PSW_LEN,
  DC_OVER_I_THR_LEN, AC_OVER_I_THR_LEN,
  EE_BAT_TYPE_LEN, BAT_MAX_VOLT_LEN, 
  EE_BAT_CAP_LEN, EE_BAT_AGE_LEN, 
  BAT_FRC_EXIT_VOLT, EE_FT_NUM_LEN,  
  EE_SOL_CAP_LEN, 
  BAT_MAINS_CHG_IN_VOLT_LEN, BAT_MAINS_CHG_OUT_VOLT_LEN,
  MORN_CHG_INH_EN_LEN, EVEN_CHG_INH_EN_LEN, EE_ABS_INTR_LEN, BAT_EQU_INTR_LEN, BAT_EQU_DUR_LEN,		
  DEV_SER_NO_LEN, DEVICE_TYPE_LEN,          
  UC_BAT_V_CONST_LEN, PAC_BAT_V_CONST_LEN,          
  PAC_BAT_CHG_I_CONST_LEN, PAC_BAT_DIS_I_CONST_LEN,  
  PAC_BAT_CHG_I_OFF_LEN, PAC_BAT_DIS_I_OFF_LEN,   
  PAC_BAT_P_CONST_LEN, UC_SOL_V_CONST_LEN,        
  PAC_SOL_V_CONST_LEN, PAC_SOL_I_CONST_LEN, PAC_SOL_I_OFF_LEN, PAC_SOL_P_CONST_LEN,                 
  UC_TEMP1_CONST_LEN, UC_TEMP2_CONST_LEN, CT_LOAD_I_OFST_LEN, CT_LOAD_I_CONST_LEN,                 
  DEV_USEABLE_SOC_MULTIPLIER_LEN, DEV_EQU_HYS_LEN,                                                     
  DEV_ABS_MF_PER_LEN, EQ_MF_PER_LEN,                         
  VFT_EXIT_HIGH_LEN, VFT_EXIT_LOW_LEN,                                 
  DEV_SYS_FLAG_LEN, DAILY_LOAD_THR_LEN,                                
  WEEKLY_LOAD_THR_LEN, NUM_DAYS_LEN, 
  USEABLE_SOC_LEN, RES_ENE_MUL_LEN, VFT_EXT_THR_LEN, 
  EVE_BAT_SOC_COR_LOW_LIM_LEN, EVE_BAT_SOC_COR_HIGH_LIM_LEN,  
  SOL_MAX_CUR_LEN, USEABLE_SOC_STRT_LEN,  
  FT_ENTRY_STRT_DAY_LEN, FT_EXIT_STRT_DAY_LEN,
  STATE_CHANGE_LOG_UNITS_LEN, STATE_CHANGE_LOG_UNITS_NO_WIFI_LEN, 
  SOL_E_CORR_LEN, SER_ASSERT_STATE_LEN, SER_STATE_ASSERTED_TIM_HR_LEN, SER_STATE_ASSERTED_TIM_MIN_LEN, 
  SER_STATE_ASSERTED_DUR_LEN, FT_EXIT_TIM_LEN, MORN_SOL_PRED_LEN, EVEN_SOL_PRED_LEN,
  DAY1_MORN_LOAD_LEN, DAY1_NGT_LOAD_LEN, 
  DAY2_MORN_LOAD_LEN, DAY2_NGT_LOAD_LEN,  
  DAY3_MORN_LOAD_LEN, DAY3_NGT_LOAD_LEN, 
  DAY4_MORN_LOAD_LEN, DAY4_NGT_LOAD_LEN, 
  DAY5_MORN_LOAD_LEN, DAY5_NGT_LOAD_LEN,       
  DAY6_MORN_LOAD_LEN, DAY6_NGT_LOAD_LEN,                                  
  DAY7_MORN_LOAD_LEN, DAY7_NGT_LOAD_LEN,  
  COMMON_THR_LEN, DEV_SOC_VOL_TABLE_LEN,                         
  DAY1_MORN_SOL_LEN, DAY1_AFT_SOL_LEN,                    
  DAY2_MORN_SOL_LEN, DAY2_AFT_SOL_LEN,                       
  DAY3_MORN_SOL_LEN, DAY3_AFT_SOL_LEN                  
};

void https_req_buff_write(https_request_buffer_body_t *param,uint8_t c){
  uint8_t i; 
  i = (param->https_request_buffer_head + 1) % HTTPS_REQ_BUFFER_SIZE;
  if (i == param->https_request_buffer_tail){
    param->https_request_buffer_tail = (param->https_request_buffer_tail + 1) % HTTPS_REQ_BUFFER_SIZE;
  }
  param->https_request_buffer[param->https_request_buffer_head] = c;
  param->https_request_buffer_head = i;
}

uint8_t https_req_buff_available(https_request_buffer_body_t *param){
	uint8_t buff_size = (HTTPS_REQ_BUFFER_SIZE + param->https_request_buffer_head - param->https_request_buffer_tail) % HTTPS_REQ_BUFFER_SIZE;
	ESP_LOGI(TAG,"Buffer size %d",buff_size);
	return buff_size;
}

uint8_t https_req_buff_peek(https_request_buffer_body_t *param){
	return(param->https_request_buffer[param->https_request_buffer_tail]);
}

void https_req_buff_flush(https_request_buffer_body_t *param){
	if(param->https_request_buffer_head != param->https_request_buffer_tail){
	   param->https_request_buffer_tail = (param->https_request_buffer_tail + 1) % HTTPS_REQ_BUFFER_SIZE;
	}
}

void detect_domain_msg(cJSON *payload){
  cJSON *readInfo = cJSON_GetObjectItem(payload, "domain");
  if(readInfo!=NULL){
    // ESP_LOGI(TAG,"Parse domain msg - %s",readInfo->valuestring);
    update_spiffs(readInfo->valuestring,domain_update);
  }
}

void httpsmsgparse(char *https_receive_buffer,https_request_buffer_body_t *param){  
  if( https_req_buff_peek(param) == update_certificate ){
    update_spiffs(https_receive_buffer,update_certificate);
  }
  else{
    cJSON *payload = cJSON_Parse(https_receive_buffer);
    ESP_LOGI(TAG,"Parsing data");
    if(detect_payload(payload)){
      ESP_LOGI(TAG,"Detected device");
      if(detect_msgType(payload)){
        ESP_LOGI(TAG,"Message type matched");
        detect_write_msg(payload);
        detect_read_msg(payload);
        detect_domain_msg(payload);
      }
    }
    cJSON_Delete(payload);
  }
}

int detect_payload(cJSON *payload){
  cJSON *device_id = cJSON_GetObjectItem(payload, "dev_id");
  if(device_id!=NULL){
    extern char dev_id[12];
    if(strcmp(device_id->valuestring,dev_id)==0){return 1;}
    else{return 0;}
  }
  else{return 0;}
}

int detect_msgType(cJSON *payload){
  cJSON *msg_type = cJSON_GetObjectItem(payload,"messagetype");
  if(msg_type!=NULL){
    if(strcmp(msg_type->valuestring,"read write response")==0){return 1;}
    else{return 0;}
  }
  else{return 0;}
}

void detect_write_msg(cJSON *payload){
  cJSON *writeInfo = cJSON_GetObjectItem(payload, "writeInfo");
  if(writeInfo!=NULL){
    parse_write_msg(writeInfo);
  }
}

void detect_read_msg(cJSON *payload){
  cJSON *readInfo = cJSON_GetObjectItem(payload, "readInfo");
  if(readInfo!=NULL){
    parse_read_msg(readInfo);
  }
}

void parse_write_msg(cJSON *writeInfo){
  ESP_LOGI(TAG,"inside parse_write_msg");
  https_req_buff_write(&https_request_buffer_body,send_response);
  char temp_addr[5];
  for(int i=0;i<sizeof(write_info_address)/sizeof(uint16_t);i++){
    sprintf(temp_addr,"%x",write_info_address[i]);
    cJSON *readvalue = cJSON_GetObjectItem(writeInfo,temp_addr);
    if(readvalue!=NULL){
      if(write_info_length[i]==4){
        ESP_LOGI(TAG,"Float: Writing to address %s and the value is %f",temp_addr,atof(readvalue->valuestring));
        ee24lc256_write_float(write_info_address[i],atof(readvalue->valuestring));
        vTaskDelay(100/portTICK_PERIOD_MS);
      }
      else{
        if(write_info_length[i]==2){
          ESP_LOGI(TAG,"16bit int: Writing to address %s and the value is %d",temp_addr,atoi(readvalue->valuestring));
          ee24lc256_write_int16(write_info_address[i],atoi(readvalue->valuestring));
          vTaskDelay(100/portTICK_PERIOD_MS);
        }
        else{
          ESP_LOGI(TAG,"8bit int: Writing to address %s and the value is %d",temp_addr,atoi(readvalue->valuestring));
          ee24lc256_write_byte(write_info_address[i],atoi(readvalue->valuestring));
          vTaskDelay(100/portTICK_PERIOD_MS);
        }
      }
    }
  }
  load_wifi_param();
  load_sensor_constants();
}

void parse_read_msg(cJSON *readInfo){
  if(parse_response(readInfo->valueint,calibration_req)){ ESP_LOGI(TAG,"parse read: calibration"); https_req_buff_write(&https_request_buffer_body,update_calibration); };
  if(parse_response(readInfo->valueint,configuration_req)){ ESP_LOGI(TAG,"parse read: configuration"); https_req_buff_write(&https_request_buffer_body,update_configuration); };
  if(parse_response(readInfo->valueint,system_thr_req)){ ESP_LOGI(TAG,"parse read: system threshold"); https_req_buff_write(&https_request_buffer_body,update_system_threshold);  };
  if(parse_response(readInfo->valueint,useable_soc_req)){ ESP_LOGI(TAG,"parse read: useable soc"); https_req_buff_write(&https_request_buffer_body,update_useable_soc); };
  if(parse_response(readInfo->valueint,factory_update)){ ESP_LOGI(TAG,"parse read: firmware update"); extern uint8_t fw_update; fw_update=1;  };
  if(parse_response(readInfo->valueint,factory_reset)){  ESP_LOGI(TAG,"parse read: factory reset"); fcn_factory_reset(); };
}