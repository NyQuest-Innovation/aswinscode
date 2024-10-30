#include "esp32s2_button.h"
#include "esp32s2_socket.h"
#include "esp32s2_buzzer.h"

#define TAG "button.c"

extern TaskHandle_t serverTask_handle;
extern TaskHandle_t clientTask_handle;
extern TaskHandle_t normalTask_handle;
extern TaskHandle_t leastTask_handle;

extern xSemaphoreHandle switchpressSemaphore;
extern xSemaphoreHandle switchpressConfirmedSemaphore;

extern uint8_t button_press;

extern uint8_t server_connect_retry,server_connect_retry_cnt;
extern uint8_t turt_state, switch_state, device_state,user_device_off;
extern uint16_t watchdog_timer_stop;

uint8_t stop_algorithm=0;

uint8_t toggle=0;
uint8_t switch_press_value=0;

static void IRAM_ATTR pushbutton_handler(void *args){
  xSemaphoreGiveFromISR(switchpressSemaphore,pdFALSE);
  gpio_isr_handler_remove(BUTTON);
}

void init_pushbutton(void){
    gpio_pad_select_gpio(BUTTON); /* Select the pin to configure */
    gpio_set_direction(BUTTON, GPIO_MODE_INPUT); /* Set the mode of the pin to be configured */
	  /* Enable pullup and disable pulldown */
    gpio_pulldown_dis(BUTTON);
    gpio_pullup_en(BUTTON);
	  /* Set interrupt type */
	  gpio_set_intr_type(BUTTON, GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON, pushbutton_handler, (void *)BUTTON);
}

uint8_t button_routine(void){
    uint16_t push_count = 0;
    uint8_t is_pressed = 0, press_count=0;
    time_t press_dur=0,elapsed_dur=0;
    time(&press_dur);
    vTaskDelay(100/portTICK_RATE_MS);
    if(gpio_get_level(BUTTON)){
      return no_press;
    }
    do{
      vTaskDelay(100/portTICK_RATE_MS);
      if(gpio_get_level(BUTTON)){
        push_count++;
        is_pressed = 0;
        press_count = 0;
        while(!is_pressed){
          vTaskDelay(100/portTICK_RATE_MS);
          if(++press_count == 4){ // if no press for 400 ms return press count
            return push_count;
          }
          if(!gpio_get_level(BUTTON)){
            vTaskDelay(50/portTICK_RATE_MS);
            is_pressed = 1;
          }
        }
      }
      time(&elapsed_dur);
    }while((elapsed_dur - press_dur)<2);
    return long_press;
}

void buttonPressDetection_routine(void *para){
  while(true){
    if(xSemaphoreTake(switchpressSemaphore, 10000 / portTICK_RATE_MS)){
      ESP_LOGI("test","press detected");
      switch_press_value=button_routine();
      if(switch_press_value!=no_press){
        xSemaphoreGive(switchpressConfirmedSemaphore);
      }
      gpio_isr_handler_add(BUTTON, pushbutton_handler, (void *)BUTTON);
    }
  }
}

void singlePressOperation(void){
  if(stop_algorithm){
    deleteConfigCalibTask();
    stop_algorithm=0;
    watchdog_timer_stop=0;
  }
  button_press=1;
  if(test_system_flag(SF_INV_FAIL)){ clear_system_flag(SF_INV_FAIL); };
  singlePressEffectOnAlgorithm();
  ESP_LOGI(TAG,"SinglePress");
}

void doublePressOperation(void){
  beep(2,SHORT_BEEP);
  blink(led_magenta_color,HALF_SECOND_BLINK);
  stop_algorithm=1;
  watchdog_timer_stop=1;
  vTaskDelay(2000/portTICK_RATE_MS);
  blink(led_magenta_color,ONE_SECOND_BLINK);
  esp_wifi_stop();
  if(switch_press_value==no_press){
    play_tone1();
    connectAP();
    extern uint8_t stopBatteryOverVoltageProtectionOnCalibration;
    stopBatteryOverVoltageProtectionOnCalibration=1;
    xTaskCreate(&tcp_server,"TCP server",5*4096,NULL,5,&serverTask_handle);   
  }
  else{
    if(switch_press_value==double_press){
      ESP_LOGI(TAG,"Double press - Double press");
      server_connect_retry=0;
      server_connect_retry_cnt=0;
      play_tone1();
      extern uint8_t stopBatteryOverVoltageProtectionOnCalibration;
      stopBatteryOverVoltageProtectionOnCalibration=1;
      xTaskCreate(&tcp_client,"TCP client",5*4096,NULL,5,&clientTask_handle);
      connectSTA("msNet","mystic123");
    }
    else{
      stop_algorithm=0;
      watchdog_timer_stop=0;
    }
  }
}

void longPressOperation(void){
  ESP_LOGI(TAG,"LongPress");
  if(turt_state == TURT_STAT_ON){
    user_device_off = 1;
    device_state=ALGO_OFF;
    turt_state=TURT_STAT_OFF;
    clear_system_flag(SF_INV_FAIL);
    extern uint8_t prev_frc_trip_count,frc_trip_count;
    frc_trip_count = prev_frc_trip_count;
    extern bool ledBlinkOn;
    ledBlinkOn=false;
    beep(4, SHORT_BEEP);
  }
  else{
    device_state=ALGO_ON;
    turt_state=TURT_STAT_ON;
    stop_algorithm=1;
    watchdog_timer_stop=1;
    vTaskDelay(1000/portTICK_PERIOD_MS);
    power_on_initialization();
    stop_algorithm=0;
    watchdog_timer_stop=0;
    user_device_off = 0;
  }   	
}

void buttonPressTask(void *para){
  while(true){
    if(switch_press_value!=no_press){
      switch(switch_press_value){
        case single_press:
            singlePressOperation();
            break;
        case double_press:
            ESP_LOGI(TAG,"DoublePress");
            switch_press_value=0;
            doublePressOperation();
            break;
        case long_press:
			      longPressOperation();
            break;
      }
      switch_press_value=0;
    }
    vTaskDelay(1000/portTICK_RATE_MS);
  }
}