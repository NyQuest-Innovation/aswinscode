#include "esp32s2_https_protocol.h"
#include "esp32s2_algorithm.h"
#include "esp32s2_data_log.h"
#include "esp32s2_parse_data.h"
#include "esp32s2_main.h"
#include "esp32s2_pac1952.h"

time_t timeinfor = 0;
#define TAG "HTTPCLIENT"

// #define log_data_payload_length 0x2FFF
uint8_t packet_length = 10;
struct FetchParms msgStruct;

algo_param_t http_data_get;

extern uint8_t server_cert_pem_start[2048];
extern uint8_t connection_stat_check,connection_stat;
extern char dev_id[12];
extern uint8_t https_password[SERV_IP_LEN];

char *https_receive_buffer = NULL;
int index_https_buffer = 0;

extern https_request_buffer_body_t https_request_buffer_body;
extern uint8_t stop_lptask;

extern uint8_t i2cFlag;
extern xSemaphoreHandle i2cSemaphore;

/*
 * HTTPS event handler
 */
esp_err_t  client_event_handler(esp_http_client_event_t *evt)
{
  assert(evt != NULL);

  switch (evt->event_id)
  {
    case HTTP_EVENT_ERROR:
        ESP_LOGI(TAG, "HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGI(TAG, "HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGI(TAG, "HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGI(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
        break;
    case HTTP_EVENT_ON_DATA:
        time(&timeinfor);
        if(stop_lptask==0){
            ESP_LOGI(TAG,"Response time is %ld",timeinfor);
            if (https_receive_buffer == NULL){
                https_receive_buffer = (char *)malloc(evt->data_len);
            }
            else{
                ESP_LOGI(TAG,"Reallocating memory");
                https_receive_buffer = (char *)realloc(https_receive_buffer, evt->data_len + index_https_buffer);
            }
            ESP_LOGI(TAG,"Reallocating memory done");
            memcpy(&https_receive_buffer[index_https_buffer], evt->data, evt->data_len);
            index_https_buffer += evt->data_len;
        }
        break;
    case HTTP_EVENT_ON_FINISH:
        if(stop_lptask==0){
            ESP_LOGI(TAG, "HTTP_EVENT_ON_FINISH");
            https_receive_buffer = (char *)realloc(https_receive_buffer, index_https_buffer + 1);
            memcpy(&https_receive_buffer[index_https_buffer], "\0", 1);
            ESP_LOGI(TAG,"%s", https_receive_buffer);
            httpsmsgparse(https_receive_buffer,&https_request_buffer_body);
            index_https_buffer=0;
            free(https_receive_buffer);
            https_receive_buffer=NULL;
        }
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
        break;
  }
  return ESP_OK;
}

void prepare_configuration_data(char *out){
    extern float max_dc_load_i,max_ac_load_i,bat_max_v;
    extern float Rated_Bat_AH,bat_frc_trip_exit_v_thr;
    extern float sol_inst, bat_mains_chg_in_v_thr, bat_vol;

    extern uint8_t battery_type, bat_age, max_frc_per_day, morn_chg_inh_flag; 
    extern uint8_t  even_chg_inh_flag, bat_abs_intr, bat_sol_equ_intr;
    extern uint8_t bat_sol_equ_dur, useable_soc_recal_dur;
    
    sprintf(out,
    "{"
    "\"dev_id\": \"%s\","
    "\"packet\": "    
    "["
    "\"%0.3f\"," /* dc overload */
    "\"%0.3f\"," /* ac overload */
    "\"%d\","    /* battery type */
    "\"%0.3f\"," /* bat max voltage */
    "\"%0.3f\"," /* bat ah capacity */
    "\"%d\","    /* bat age */
    "\"%0.3f\"," /* frc trip exit vol */
    "\"%d\","    /* no of ft per day */
    "\"%0.3f\"," /* sol capacity */
    "\"%0.3f\"," /* mains charge in voltage */
    "\"%0.3f\"," /* battery voltage */
    "\"%d\","    /* morn chg inh flag */
    "\"%d\","    /* even chg inh flag */
    "\"%d\","    /* per abs int */
    "\"%d\","    /* per equ int */
    "\"%d\","    /* per equ dur */
    "\"%d\""     /* per use soc */
    "]}",dev_id,max_dc_load_i,max_ac_load_i,battery_type,bat_max_v,Rated_Bat_AH,bat_age,
    bat_frc_trip_exit_v_thr, max_frc_per_day, sol_inst, bat_mains_chg_in_v_thr,
    bat_vol, morn_chg_inh_flag, even_chg_inh_flag, bat_abs_intr, bat_sol_equ_intr,
    bat_sol_equ_dur, useable_soc_recal_dur);
}

void prepare_calibration_data(char *out){
    sprintf(out,
    "{"
    "\"dev_id\": \"%s\","
    "\"packet\": "    
    "["
    "\"%0.3f\","
    "\"%0.3f\","
    "\"%0.3f\","
    "\"%0.3f\","
    "\"%0.3f\","
    "\"%0.3f\","
    "\"%0.3f\","
    "\"%0.3f\","
    "\"%0.3f\","
    "\"%0.3f\","
    "\"%0.3f\","
    "\"%0.3f\","
    "\"%0.3f\","
    "\"%0.3f\","
    "\"%d\","
    "\"%0.3f\""
    "]}",dev_id,uc_bat_v_const,pac_bat_v_const,pac_bat_chg_i_const,pac_bat_dis_i_const,pac_bat_chg_i_offset,pac_bat_dis_i_offset,pac_bat_p_const,
    pac_sol_v_const,uc_sol_v_const,pac_sol_i_const,pac_sol_i_offset,pac_sol_p_const,uc_temp1_const,uc_temp2_const,ct_load_i_offset,ct_load_i_const );    
}

void prepare_systhr_data(char *out){
    sprintf(out,
    "{"
    "\"dev_id\": \"%s\","
    "\"packet\": "    
    "["
    "\"%d\","
    "\"%0.3f\","
    "\"%0.3f\","
    "\"%0.3f\","
    "\"%0.3f\""
    "]}",dev_id,dev_state_flag,useable_SOC1,bat_frc_trip_exit_v_thr,eve_soc_corr,night_soc_corr); 
}

void prepare_useable_soc_data(char *out){
    sprintf(out,
    "{"
    "\"dev_id\": \"%s\","
    "\"packet\": "    
    "["
    "\"%02d-%02d-%02d %02d:%02d:%02d\","
    "\"%0.3f\","
    "\"%0.3f\""
    "]}",dev_id,load.date_time[0],load.date_time[1],load.date_time[2],load.date_time[3],load.date_time[4],dev_state_flag,useable_SOC1,bat_frc_trip_exit_v_thr);     
}

void prepare_LOGDATA(char *out,uint8_t cond){
    time(&timeinfor);
    date_time = localtime(&timeinfor);
    sprintf(out,
    "{"
    "\"token\": \"12345678\","
    "\"dev_id\": \"%s\","
    "\"messagetype\": \"log\","
    "\"readflag\": 0,"
    "\"packet\": [",dev_id    
    );
    if(cond){
        for(int i=0;i<packet_length;i++){
            /*Add EEPROM read here*/
            xSemaphoreTake(i2cSemaphore,portMAX_DELAY);
            turtle_log_get((uint8_t *)&http_data_get,i);
            xSemaphoreGive(i2cSemaphore);
            sprintf(out+strlen(out),
            "["
            "\"%s\","
            "\"%02d-%02d-%02d %02d:%02d:%02d\","
            "\"%0.2f\","
            "\"%0.2f\","
            "\"%0.2f\","
            "\"%0.2f\","
            "\"0.04\","
            "\"0.76\","	
            "\"%0.2f\","
            "\"1.38\","
            "\"%0.2f\","	
            "\"%d\","
            "\"%0.2f\","
            "\"%d\","
            "\"%0.2f\","
            "\"%0.2f\","
            "\"%0.2f\","
            "\"%0.2f\","
            "\"%0.2f\","	
            "\"%d\","
            "\"%d\","
            "\"%d\","
            "\"%d\","
            "\"%d\","
            "\"%0.2f\","
            "\"%d\","
            "\"%d\","
            "\"%d\","
            "\"%0.2f\"]",dev_id,(http_data_get.date_time[2])+1900,http_data_get.date_time[1],http_data_get.date_time[0],http_data_get.date_time[3],http_data_get.date_time[4],http_data_get.date_time[5],http_data_get.cur_bat_v,
            http_data_get.cur_chg_i,http_data_get.cur_dis_i,http_data_get.cur_sol_i,http_data_get.cur_sol_v,http_data_get.cur_ac_load_i,http_data_get.cur_temp,http_data_get.bat_cur_cap_coul,
            http_data_get.dev_algo_state,http_data_get.sol_e,http_data_get.sol_bat_e,http_data_get.bat_dis_e,http_data_get.bat_ac_chg_e,http_data_get.ac_load_e,http_data_get.num_sum_samples,
            http_data_get.log_type,http_data_get.state_change_reason,http_data_get.dev_stat,http_data_get.algo_stat,http_data_get.cur_inv_v,http_data_get.error,http_data_get.daystartflag,
            http_data_get.resetflag,http_data_get.utility_savings
            );
            if(i<packet_length-1)
            {
                strcat(out,",");
            }
        }
    }
    sprintf(out+strlen(out),
    "]"
    "}"
    );
    ESP_LOGI("TAG"," legnth is %d",strlen(out));
    ESP_LOGI(TAG,"Data to be sent %s",out);
}

void test_server_connection(void){
    msgStruct.OnGotData = NULL;
    msgStruct.method = Post;
    msgStruct.headerCount = 0;     
    char buffer[0x2FFF];
    prepare_LOGDATA(buffer,0);
    msgStruct.body = buffer;
    char url[2048];
    read_domain(url);
    strcat(url,"/api/log/");
    fetch(url, &msgStruct);
}

void send_DATA_to_server(void){
    msgStruct.OnGotData = NULL;
    msgStruct.method = Post;
    msgStruct.headerCount = 0;     
    char buffer[0x2FFF];
    prepare_LOGDATA(buffer,1);
    msgStruct.body = buffer;
    char url[2048];
    read_domain(url);
    strcat(url,"/api/log/");
    ESP_LOGI("inside sendDataToServer","%s",url);
    fetch(url, &msgStruct);
}

void bidirectional_communication_commands(https_request_buffer_body_t *param){
    msgStruct.OnGotData = NULL;
    msgStruct.method = Post;
    msgStruct.headerCount = 0;     
    char buffer[0x2FFF];
    char url[2048];
    ESP_LOGI(TAG,"Data is %d",https_req_buff_peek(param));
    switch(https_req_buff_peek(param)){
        case update_certificate:
            read_domain(url);
            strcat(url,"/api/getCert/");
            msgStruct.method = Get;
            fetch(url,&msgStruct);
            break;

        case get_information:
            sprintf(buffer,"{\"dev_id\":\"%s\"}",dev_id);
            ESP_LOGI(TAG,"Msg sent is %s",buffer);
            msgStruct.body = buffer;
            read_domain(url);
            strcat(url,"/api/getReadWriteInfo/");
            fetch(url, &msgStruct);
            break;

        case update_calibration:
            prepare_calibration_data(buffer);
            ESP_LOGI(TAG,"Msg sent is %s",buffer);
            msgStruct.body = buffer;
            read_domain(url);
            strcat(url,"/api/updateCalibration/");
            fetch(url, &msgStruct);
            break;

        case update_configuration:
            prepare_configuration_data(buffer);
            ESP_LOGI(TAG,"Msg sent is %s",buffer);
            msgStruct.body = buffer;
            read_domain(url);
            strcat(url,"/api/updateConfig/");
            fetch(url, &msgStruct);
            break;

        case update_system_threshold:
            prepare_systhr_data(buffer);
            ESP_LOGI(TAG,"Msg sent is %s",buffer);
            msgStruct.body = buffer;
            read_domain(url);
            strcat(url,"/api/updateThreshold/");
            fetch(url, &msgStruct);
            break;

        case send_response:
            sprintf(buffer,"{\"dev_id\":\"%s\"}",dev_id);
            ESP_LOGI(TAG,"Msg sent is %s",buffer);
            msgStruct.body = buffer;
            read_domain(url);
            strcat(url,"/api/updateWriteStatus/");
            fetch(url, &msgStruct);            
            break;

        case update_useable_soc:
            prepare_useable_soc_data(buffer);
            ESP_LOGI(TAG,"Msg sent is %s",buffer);
            msgStruct.body = buffer;
            read_domain(url);
            strcat(url,"/api/updateSOC/");
            fetch(url, &msgStruct);        
            break;
    }
}

void fetch(char *url, struct FetchParms *fetchParms){
    read_spiffs((char *)server_cert_pem_start,read_certificate);    

    esp_http_client_config_t clientConfig = {
      .url = url,
      .event_handler = client_event_handler,
      .cert_pem = (char *)server_cert_pem_start,
      .username = (char *)dev_id,
      .password = (char *) https_password,
      .auth_type = HTTP_AUTH_TYPE_BASIC,
    };

    ESP_LOGI(TAG,"Reached point 1");
    esp_http_client_handle_t client = esp_http_client_init(&clientConfig);
    ESP_LOGI(TAG,"Reached point 2");
    
    if (fetchParms->method == Post){
        esp_http_client_set_method(client, HTTP_METHOD_POST);
    }

    for (int i = 0; i < fetchParms->headerCount; i++){
        esp_http_client_set_header(client, fetchParms->header[i].key, fetchParms->header[i].val);
    }

    if(fetchParms->body != NULL){
        esp_http_client_set_post_field(client, fetchParms->body, strlen(fetchParms->body));
    }

    esp_err_t err = esp_http_client_perform(client);
    fetchParms->status = esp_http_client_get_status_code(client); 

    ESP_LOGI(TAG, "HTTP GET status = %d",fetchParms->status);
    
    if (err == ESP_OK){
        ESP_LOGI(TAG, "HTTP GET status = %d",
        esp_http_client_get_status_code(client));
        if((fetchParms->status>=200)&&(fetchParms->status<400)){
            if(connection_stat_check){
                connection_status_obtained=1;
                connection_stat_check=0;
                connection_stat=2;
                blink(led_green_color,LED_OFF);
                vTaskDelay(3000/portTICK_PERIOD_MS);
                extern uint8_t stop_algorithm;
                stop_algorithm=0;
                extern uint16_t watchdog_timer_stop;
                watchdog_timer_stop=0;
            }
            else{
                extern operations_parameters_t least_priority_operations_parameters;
                least_priority_operations_parameters.status=lpos_response_received;
                least_priority_operations_parameters.wait_time=0;
            }
        }
    }
    else{
        ESP_LOGE(TAG, "HTTP GET request failed: %s", esp_err_to_name(err));
        ESP_LOGI(TAG,"HTTP password - %s\n",(char *)https_password);
        if(connection_stat_check){
            connection_status_obtained=1;
            connection_stat_check=0;
            connection_stat=2;
            blink(led_blue_color,LED_OFF);
            vTaskDelay(3000/portTICK_PERIOD_MS);
            extern uint8_t stop_algorithm;
            stop_algorithm=0;
            extern uint16_t watchdog_timer_stop;
            watchdog_timer_stop=0;
        }
    }
 
    esp_http_client_cleanup(client);
}
