/*
  ESP-IDF VERSION :- 4.3.1
 */
#include <stdio.h>
#include "esp32s2_task.h"
#include "esp32s2_socket.h"
#include "esp_spiffs.h"
#include "esp32s2_buzzer.h"
#include "esp32s2_parse_data.h"
#include "esp_ota_ops.h" 
#include "esp_partition.h"
#include "esp32s2_pac1952.h"

uint8_t server_cert_pem_start[2048];

#define TAG "inside main"

void fcn_factory_reset(void){
  esp_partition_iterator_t pi = esp_partition_find(ESP_PARTITION_TYPE_APP,ESP_PARTITION_SUBTYPE_APP_FACTORY ,"factory");
  if( pi != NULL ){
    const esp_partition_t * factory = esp_partition_get ( pi );
    esp_partition_iterator_release ( pi );
    if(esp_ota_set_boot_partition(factory)==ESP_OK) esp_restart () ;
  }  
}

void init_spiffs(void){
  esp_vfs_spiffs_conf_t conf = {
    .base_path = "/spiffs",.partition_label = NULL,
    .max_files = 5,.format_if_mount_failed = true
  };
  esp_err_t ret = esp_vfs_spiffs_register(&conf);
  if (ret!=ESP_OK){
    if (ret == ESP_FAIL){ ESP_LOGE(TAG, "Failed to mount or format filesystem"); } 
    else if(ret == ESP_ERR_NOT_FOUND){ ESP_LOGE(TAG, "Failed to find SPIFFS partition"); }
    else{ ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret)); }
    return;
  }
  size_t total = 0, used = 0;
  ret = esp_spiffs_info(NULL, &total, &used);
  if(ret != ESP_OK){ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));} 
  else{ ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used); }
}

char urls[11][32] = { 
  "/spiffs/getReadWrite.txt", 
  "/spiffs/updateCalib.txt", 
  "/spiffs/updateConfig.txt",
  "/spiffs/updateThreshold.txt",
  "/spiffs/updateStatus.txt",
  "/spiffs/updateSoC.txt",
  "/spiffs/updateLog.txt",
  "/spiffs/getCert.txt",
  "/spiffs/google.cer",
  "/spiffs/updateOTA.txt",
  "/spiffs/domain.txt"
};

void read_domain(char *ptr){
  bzero(ptr,2048);
  ESP_LOGI(TAG, "reading from domain.txt");
  FILE* f = fopen("/spiffs/domain.txt", "r");
  fseek(f, 0, SEEK_END);
  long fsize = ftell(f);
  fseek(f, 0, SEEK_SET);  /* same as rewind(f); */
  fread(ptr, 1, fsize, f);
  fclose(f);
  ptr[fsize] = 0;
  #warning "Delete following line after server fix"
  memcpy(ptr,"https://log.nyquestapi.com",strlen("https://log.nyquestapi.com"));
  ESP_LOGI(TAG,"Data read is \n %s",ptr);
}

void read_spiffs(char *ptr, int type ){
  bzero(ptr,2048);
  ESP_LOGI(TAG, "reading from %s",urls[type]);
  FILE* f = fopen(urls[type], "r");
  fseek(f, 0, SEEK_END);
  long fsize = ftell(f);
  fseek(f, 0, SEEK_SET);  /* same as rewind(f); */
  fread(ptr, 1, fsize, f);
  fclose(f);
  ptr[fsize] = 0;
  ESP_LOGI(TAG,"Data read is \n %s",ptr);
}

void update_spiffs( char *param, int type ){
  ESP_LOGI(TAG, "updating from %s",urls[type]);
  remove(urls[type]);
  FILE* f = fopen(urls[type], "w");
  if(f==NULL){ESP_LOGE(TAG, "Failed to open %s",urls[type]);return;}
  fprintf(f,"%s",param);
  fclose(f);
  if(type==update_certificate){ read_spiffs((char *)server_cert_pem_start,read_certificate); };
}

void app_main(void){
  ESP_LOGI(TAG,"Hardware version : %x ; Software version : %x",HW_VERSION,FW_VERSION);
  peripheral_initializaton();
  watchdogTask();
}

void peripheral_initializaton(void){
  #if ICUBE
  #warning "Device configured as iCUBE"
  #else
  #warning "Device configured as iCON"
  #endif  
  init_buzzer();
  init_led();         
  init_i2c();        
  init_switches();    
#if ICUBE
  S4_SW_init();
#endif 
  init_pushbutton();
  init_adc();         
  init_mains_sense(); 
  initializeBatteryOvervoltageSense();
  init_pac1952();   
  check_time();
  init_spiffs();
  read_spiffs((char *)server_cert_pem_start,read_certificate);
  power_on_initialization();
  esp32s2_WiFi_Init();
  createSemaphores();
  createTasks();
}

void init_data(void){
  algo_param.cur_bat_v           = 13.1;
  algo_param.cur_chg_i           = 5.0;
  algo_param.cur_dis_i           = 0.0;
  algo_param.cur_sol_v           = 13.1;
  algo_param.cur_sol_i           = 5.0;
  algo_param.cur_ac_load_i       = 3.0;
  algo_param.cur_temp            = 35;
  algo_param.bat_cur_cap_coul    = 93.0;
  algo_param.dev_algo_state      = 3;
  algo_param.sol_e               = 12;
  algo_param.sol_bat_e           = 12;
  algo_param.bat_dis_e           = 12;
  algo_param.bat_ac_chg_e        = 12;  
  algo_param.ac_load_e           = 12;
  algo_param.num_sum_samples     = 1800;
  algo_param.log_type            = 3;
  algo_param.state_change_reason = 0;
  algo_param.dev_stat            = 48;
  algo_param.algo_stat           = 14336;
  algo_param.cur_inv_v           = 14.2;
  algo_param.error               = 10;
  algo_param.daystartflag        = 1;
  algo_param.resetflag           = 0;
  algo_param.utility_savings     = 0.25;  
}