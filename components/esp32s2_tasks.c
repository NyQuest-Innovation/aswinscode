#include "esp32s2_task.h"
#include "esp32s2_wifi.h"
#include "esp32s2_https_protocol.h"
#include "esp32s2_i2c_protocol.h"
#include "esp32s2_gpio.h"
#include "esp32s2_24lc512.h"
#include "esp32s2_socket.h"
#include "esp32s2_adc.h"
#include "esp32s2_algorithm.h"
#include "esp32s2_data_log.h"
#include "esp32s2_main.h"
#include "esp32s2_ina226.h"
#include "esp32s2_pac1952.h"
#include "esp32s2_buzzer.h"
#include "esp32s2_parse_data.h"
#include "esp32s2_button.h"
#define TAG "esp32s2_tasks.c"
#include "freertos/event_groups.h"

extern uint8_t server_cert_pem_start[2048];
extern esp_err_t  client_event_handler(esp_http_client_event_t *evt);
extern uint8_t ap_ssid_buff[AP_SSID_LEN], ap_psw_buff[AP_PSW_LEN];
extern uint8_t connection_stat_check, config_calib_mode;
extern uint8_t stop_algorithm, packet_length;
extern char dev_id[12];

xSemaphoreHandle serverReset;
xSemaphoreHandle factoryReset;
xSemaphoreHandle otaSemaphore;
xSemaphoreHandle logdataSemaphore;
xSemaphoreHandle connectionSemaphore;
xSemaphoreHandle switchpressSemaphore;
xSemaphoreHandle switchpressConfirmedSemaphore;
xSemaphoreHandle serverSemaphore;
xSemaphoreHandle testConnectionSemaphore;
xSemaphoreHandle pacAlertSemaphore;
xSemaphoreHandle i2cSemaphore;
xSemaphoreHandle ledTaskSemaphore;
xSemaphoreHandle overVoltageProtectionSemaphore;
xSemaphoreHandle stateMachineSemaphore;
xSemaphoreHandle dataLoggingSemaphore;

TaskHandle_t serverTask_handle = NULL;
TaskHandle_t clientTask_handle = NULL;
TaskHandle_t normalTask_handle = NULL;
TaskHandle_t leastTask_handle  = NULL;

EventGroupHandle_t task_watchdog;


uint8_t https_request, https_action=0;
https_request_buffer_body_t https_request_buffer_body = {0,0,0};

uint8_t exit_server_task, access_point;
uint8_t algorithm_count=0,rtc_sync=0;
bool wifi_connection_status = 0;

uint8_t server_connect_retry=0,server_connect_retry_cnt=0;
uint8_t turt_state = TURT_STAT_ON, switch_state = 0, device_state=ALGO_ON,user_device_off=0;
uint8_t wait_time_sync=0;
uint8_t connection_stat_check=0,connection_stat=0,connection_stat_check_cnt=0;
uint16_t configuration_timeout_cnt;

operations_parameters_t critical_operations_parameters;
operations_parameters_t normal_operations_parameters;
operations_parameters_t least_priority_operations_parameters;

uint8_t fw_update=0,stop_lptask=0,perform_ota_task=0;

uint8_t i2cFlag=0;

uint16_t watchdog_timer_count=0, watchdog_timer_reset=12000, watchdog_timer_stop=1;

uint8_t stopBatteryOverVoltageProtectionOnCalibration=0;

bool ledBlinkOn=false;

void createSemaphores(void){
  serverReset = xSemaphoreCreateBinary();
  factoryReset = xSemaphoreCreateBinary();
  otaSemaphore = xSemaphoreCreateBinary();
  logdataSemaphore = xSemaphoreCreateBinary();
  connectionSemaphore = xSemaphoreCreateBinary();
  switchpressSemaphore = xSemaphoreCreateBinary();
  switchpressConfirmedSemaphore = xSemaphoreCreateBinary();
  serverSemaphore = xSemaphoreCreateBinary();
  testConnectionSemaphore = xSemaphoreCreateBinary();
  pacAlertSemaphore = xSemaphoreCreateBinary();
  ledTaskSemaphore = xSemaphoreCreateBinary();
  overVoltageProtectionSemaphore = xSemaphoreCreateBinary();
  stateMachineSemaphore = xSemaphoreCreateBinary();
  dataLoggingSemaphore = xSemaphoreCreateBinary();
  i2cSemaphore = xSemaphoreCreateBinary();
  task_watchdog = xEventGroupCreate();
}

void test_wifi_config(void *param){
  while(true){
    if(xSemaphoreTake(testConnectionSemaphore, 10000 / portTICK_RATE_MS)){
      blink(led_green_color,ONE_SECOND_BLINK);
      deleteConfigCalibTask();
      vTaskDelay(1000/portTICK_PERIOD_MS);
      connection_stat_check=1;
      connectSTA( (char *) ap_ssid_buff , (char *) ap_psw_buff );
    }
  }
}

void mainTimerTask( void * param){
  static uint8_t ledTaskCount=0,overVoltageTaskCount=0,stateMachineTaskCount=0;
  static uint16_t dataLoggingTaskCount=0;
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 1;
  xLastWakeTime = xTaskGetTickCount();
  while(1){
    if(++ledTaskCount>10){
      ledTaskCount=0; 
      xSemaphoreGive(ledTaskSemaphore);
    }
    if(++overVoltageTaskCount>30){
      overVoltageTaskCount=0; 
      xSemaphoreGive(overVoltageProtectionSemaphore);
    }
    if(++stateMachineTaskCount>100){
      stateMachineTaskCount=0;
      xSemaphoreGive(stateMachineSemaphore); 
    }
    if(++dataLoggingTaskCount>1000){
      dataLoggingTaskCount=0; 
      xSemaphoreGive(dataLoggingSemaphore);
    }        
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void overvoltageProtectionTask( void * param){
    while(1){
      xSemaphoreTake(overVoltageProtectionSemaphore,portMAX_DELAY);
      if(!test_error((ERR_NOT_CALIB | ERR_NOT_CONFIG))){
        if(ledBlinkOn){
          if(stopBatteryOverVoltageProtectionOnCalibration==0){
            check_battery_voltage(); 
          }
        }
      }
      xEventGroupSetBits(task_watchdog,overvoltageProtectionTaskEvent);
    }
}

void ledBlinkTask( void * param){
    while(1){
      xSemaphoreTake(ledTaskSemaphore,portMAX_DELAY);
      if(ledBlinkOn){
        led_blink();
      }
    }
}

void stateMachineTask( void * param){
    while(1){
      xSemaphoreTake(stateMachineSemaphore,portMAX_DELAY);

      if(test_common_flag(CF_RESET_DEV)){
        clear_common_flag(CF_CLR_ALL);
        xSemaphoreGive(serverReset);
      }
 
      server_connect_retry_cnt+=1;
 
      ESP_LOGI(TAG,"Heap value %d",xPortGetFreeHeapSize());
      if(stop_algorithm==0){
        if(!test_error((ERR_NOT_CALIB | ERR_NOT_CONFIG))){
          ESP_LOGI(TAG,"iCON LV 8.00 algorithm");
          turtle_algorithm();
          watchdog_timer_count=0;
        }
        else{
          watchdog_timer_count=0;
          if(test_error(ERR_NOT_CALIB)){
            ESP_LOGI(TAG,"Device not calibrated");
          }

          if(test_error(ERR_NOT_CONFIG)){
            ESP_LOGI(TAG,"Device not configured");
          }
        }
      }
#if ICUBE==0
      #warning "Inverter failure buzzer enabled"
      if(test_system_flag(SF_INV_FAIL)){ toggle_buzzer(); };
#endif 
      xEventGroupSetBits(task_watchdog,stateMachineTaskEvent);
    }
}

void OnConnected(void *para){
  while(true){
    if(xSemaphoreTake(logdataSemaphore, 10000 / portTICK_RATE_MS)){
      if(connection_stat_check){
        test_server_connection();
      }
      else{
        if(https_action==1){
          bidirectional_communication_commands(&https_request_buffer_body);
        }
        else{
          send_DATA_to_server();
        }
      }
      ESP_LOGI("inside on connected", "Done!");
    }
  }
}

void dataLoggingTask( void * param){
  while(1){
    xSemaphoreTake(dataLoggingSemaphore,portMAX_DELAY);
    xEventGroupSetBits(task_watchdog,dataLoggingTaskEvent);

    if(watchdog_timer_stop==0){
      ESP_LOGI(TAG,"Checking watchdogTimer");
      if(++watchdog_timer_count>2){
        ESP_LOGI(TAG,"Watchdog Timer Expired");
        switch_state_4_5();
        esp_restart();  
      }
    }

    static uint8_t getInfoTime=0;
    if(++getInfoTime>30){
      getInfoTime=0;
      https_req_buff_write(&https_request_buffer_body,get_information);
    }

    if((stop_algorithm==0)&&(stop_lptask==0)){
      ESP_LOGI(TAG,"10 second tick");
      if(!test_error((ERR_NOT_CALIB | ERR_NOT_CONFIG))){
        least_priority_operations();
      }
    }

    if(connection_stat_check){
      if(++connection_stat_check_cnt>6){
        connection_stat_check=0;
        stop_algorithm=0;
        watchdog_timer_stop=0;
      }
    }
    if(stop_algorithm==1){
      if(++configuration_timeout_cnt>=30){
        configuration_timeout_cnt=0;
        stop_algorithm=0;
        watchdog_timer_stop=0;
        deleteConfigCalibTask();
      }
    }


  }
}

void TrySyncTime(void){
  if(rtc_sync==0){
    ESP_LOGI(TAG,"Trying to sync time");
    esp_wifi_stop();
    if(access_point==primary){
      ESP_LOGI(TAG,"Connecting to primary access point");
      connectSTA((char *)ap_ssid_buff,(char *)ap_psw_buff);
    }
    else{
      ESP_LOGI(TAG,"Connecting to secondary access point");
      connectSTA("E24by7","Energy24by7");
    }
    least_priority_operations_parameters.status=waiting_for_time_synced;
  }
  else{
    wait_time_sync=0;
    least_priority_operations_parameters.status=lpos_data_not_available;    
  }
}

void TimeoutSyncTime(void){
  if(rtc_sync==0){
    ESP_LOGI(TAG,"Waiting to sync time");
    if(++wait_time_sync>2){
      wait_time_sync=0;
      least_priority_operations_parameters.status=time_not_synced;
      if(access_point==primary){
        access_point=secondary;
      }
      else{
        access_point=primary;
      }
      ESP_LOGI(TAG,"Couldn't sync time");
    }
  }
  else{
    wait_time_sync=0;
    least_priority_operations_parameters.status=lpos_data_not_available;
  }  
}

void checkDataAvailable(void){
  if(fw_update){
      ESP_LOGI(TAG,"OTA update");
      fw_update=0;
      perform_ota_task=1;
      least_priority_operations_parameters.status=lpos_data_available;      
  }
  else{
    if(turtle_log_available()>packet_length){
      ESP_LOGI(TAG,"Data available in EEPROM");
      least_priority_operations_parameters.status=lpos_data_available;
    }
    else{
      if(https_req_buff_available(&https_request_buffer_body)!=0){
        ESP_LOGI(TAG,"Request is %d",https_req_buff_peek(&https_request_buffer_body));
        https_action=1;
        least_priority_operations_parameters.status=lpos_data_available;
      }
      else{
        ESP_LOGI(TAG,"No operation to perform");
      }
    }    
  }
}

void connectingToAP(void){
  if(wifi_connection_status){
    ESP_LOGI(TAG,"Already connected to AP");
    least_priority_operations_parameters.status=lpos_link_connected;
  }
  else{
    ESP_LOGI(TAG,"Not connected to AP");
    least_priority_operations_parameters.status=lpos_link_not_connected;
  }
}

void trySwitchingAP(void){
  if(access_point==primary){
    ESP_LOGI(TAG,"Connecting to primary access point");
    connectSTA((char *)ap_ssid_buff,(char *)ap_psw_buff);
  }
  else{
    ESP_LOGI(TAG,"Connecting to secondary access point");
    connectSTA("E24by7","Energy24by7");
  }
  least_priority_operations_parameters.status=lpos_link_connected;
  least_priority_operations_parameters.wait_time = 0;
}

void waitConnectingToAP(void){
  if(wifi_connection_status==0){
    if(++least_priority_operations_parameters.wait_time>1){
      esp_wifi_stop();
      if(access_point==primary){
        access_point=secondary;
      }else{
        access_point=primary;
      }
      least_priority_operations_parameters.wait_time=0;
      least_priority_operations_parameters.status=lpos_link_not_connected;
    }
    else{
      if(access_point==primary){
        ESP_LOGI(TAG,"Wait to connect with primary access point");
      }
      else{
        ESP_LOGI(TAG,"Wait to connect with secondary access point");
      }
    }
  }
  else{
    if(access_point==primary){
      ESP_LOGI(TAG,"Connected to primary access point");
    }
    else{
      ESP_LOGI(TAG,"Connected to secondary access point");
    }
    if(perform_ota_task){
      stop_lptask=1;
      perform_ota_task = 0;
      fw_update=0;
      xSemaphoreGive(otaSemaphore);
    }
    else{
      xSemaphoreGive(logdataSemaphore);
    }
    least_priority_operations_parameters.wait_time=0;
    least_priority_operations_parameters.status=lpos_waiting_for_response;
  }  
}

void waitingForServerResponse(void){
  if(++least_priority_operations_parameters.wait_time>12){
    least_priority_operations_parameters.status=lpos_communication_failed;
    least_priority_operations_parameters.wait_time=0;
    ESP_LOGI(TAG,"Data logging failed");
  }
  else{
    ESP_LOGI(TAG,"Waiting for response from server");
  }
}

void onResponseFromServer(void){
  ESP_LOGI(TAG,"Received response from server");
  least_priority_operations_parameters.wait_time=0;
  if(https_action==1){
    https_req_buff_flush(&https_request_buffer_body);
    https_action=0;
  }
  else{
    ESP_LOGI(TAG,"EEPROM tail updated");
    turtle_log_update_tail();
  }
  least_priority_operations_parameters.status=lpos_data_not_available;
  esp_wifi_stop();
}

void onCommunicationFailed(void){
  least_priority_operations_parameters.status=lpos_data_not_available;
  esp_wifi_stop();
}

void least_priority_operations(void){
  switch(least_priority_operations_parameters.status){
    case time_not_synced:
        TrySyncTime();
        break;
    case waiting_for_time_synced:
        TimeoutSyncTime();
        break;
    case lpos_data_not_available:
        checkDataAvailable();
        break;
    case lpos_data_available:
        connectingToAP();
        break;
    case lpos_link_not_connected:
        trySwitchingAP();
        break;
    case lpos_link_connected:
        waitConnectingToAP();
        break;
    case lpos_waiting_for_response:
        waitingForServerResponse();
        break;
    case lpos_response_received:
        onResponseFromServer();
        break;
    case lpos_communication_failed:
        onCommunicationFailed();
        break;
  }
}

void perform_ota(void *params){
  extern uint8_t https_password[SERV_IP_LEN];
  while (true){
    char url[2048];
    read_spiffs(url,ota_update);
    xSemaphoreTake(otaSemaphore, portMAX_DELAY);
    ESP_LOGI(TAG, "Invoking OTA");
    esp_http_client_config_t clientConfig = {
      .url = url,
      .event_handler = client_event_handler,
      .cert_pem = (char *)server_cert_pem_start,
      .username = (char *)dev_id,
      .password = (char *) https_password,
      .auth_type = HTTP_AUTH_TYPE_BASIC,
    };
    if(esp_https_ota(&clientConfig) == ESP_OK){
      printf("OTA flash succsessfull");
      printf("restarting in 5 seconds\n");
      stop_algorithm=1;
      watchdog_timer_stop=1;
      vTaskDelay(pdMS_TO_TICKS(5000));
      switch_state_4_5();
      esp_restart();
    }
    stop_lptask=0;
    fw_update=1;
    ESP_LOGE(TAG,"Failed to update firmware");
  }  
}

void perform_factory_reset(void *params){
  while(1){
    xSemaphoreTake(factoryReset, portMAX_DELAY);
    extern void fcn_factory_reset(void);
    fcn_factory_reset();
  }
}

void perform_system_reset(void *params){
  while(1){
    xSemaphoreTake(serverReset,portMAX_DELAY);
    i2c_eeprom_write_buff(0xFE7F,(uint8_t *) &common_flag,2);
    printf("restarting in 5 seconds\n");
    stop_algorithm=1;
    watchdog_timer_stop=1;
    vTaskDelay(pdMS_TO_TICKS(5000));
    switch_state_4_5();
    esp_restart();  
  }
}

void watchdogTask(void){
  while(1){

    if(watchdog_timer_stop==0){
      uint32_t result = xEventGroupWaitBits(task_watchdog, allEvents, pdTRUE, pdTRUE, 10000/portTICK_RATE_MS);
      if((result&allEvents)==allEvents){
        ESP_LOGI("Inside watchdogTask","System healthy");
      }
      else{

        ESP_LOGI(TAG,"Watchdog Timer Expired");
        switch_state_4_5();

        if(!(result&overvoltageProtectionTaskEvent)){
          ESP_LOGI("Inside watchdogTask","overvoltage protection task not responding");
        }

        if(!(result&stateMachineTaskEvent)){
          ESP_LOGI("Inside watchdogTask","state machine task not responding");
        }

        if(!(result&dataLoggingTaskEvent)){
          ESP_LOGI("Inside watchdogTask","data logging task not responding");
        }

        esp_restart();
      }
    }
    vTaskDelay(1000/portTICK_RATE_MS);
  }
}

void createTasks(void){
  xTaskCreate(&buttonPressDetection_routine,"Button press",4096,NULL,5,NULL);
  xTaskCreate(&buttonPressTask,"Button press confirmed",4096,NULL,5,NULL);
  xTaskCreate(&dataLoggingTask,"dataLogging",4096,NULL,1,NULL);
  xTaskCreate(&ledBlinkTask,"ledBlink",1024,NULL,3,NULL);
  xTaskCreate(&overvoltageProtectionTask,"overvoltageProtection",4096,NULL,4,NULL);
  xTaskCreate(&stateMachineTask,"stateMachineAlgorithm",4096,NULL,2,NULL);
  xSemaphoreGive(i2cSemaphore);
  xTaskCreate(&mainTimerTask,"mainTimer",1024,NULL,5,NULL);

  if(test_error(ERR_NO_MEMORY)){
    ESP_LOGI(TAG,"EEPROM not detected");
    blink(led_red_color,LED_OFF);
  }
  else{
    least_priority_operations_parameters.status=time_not_synced;
    https_req_buff_write(&https_request_buffer_body,update_configuration);
    watchdog_timer_stop=0;
    xTaskCreate(&OnConnected, "data logging", 5*4096, NULL, 5, NULL);
    xTaskCreate(&test_wifi_config,"Test configuration",4096,NULL,5,NULL);
    xTaskCreate(perform_ota, "perform_ota", 1024 * 8, NULL, 2, NULL);
    xTaskCreate(perform_factory_reset, "perform_fac_reset", 4096, NULL, 2, NULL);
    xTaskCreate(perform_system_reset, "perform_system_reset", 4096 , NULL, 2, NULL);
  }
}