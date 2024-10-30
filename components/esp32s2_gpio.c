#include "esp32s2_gpio.h"
#include "esp32s2_algorithm.h"
#include "esp32s2_buzzer.h"
#include "esp32s2_main.h"

int buzzer_state = 0;
int led_pins[3] = {LED_RED,LED_GREEN,LED_BLUE};
int switch_pins[4]={BAT_MOS_SW,SOL_MOS_SW,M_RELAY,TRIAC_SW};
uint8_t switch_state;

#define GPIO_INPUT_PIN_SEL  (1ULL<<ALERT)

void initializeBatteryOvervoltageSense(void){
    gpio_config_t gpio_conf ={
      .intr_type = GPIO_INTR_DISABLE,
      .pin_bit_mask = GPIO_INPUT_PIN_SEL,
      .mode = GPIO_MODE_INPUT,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    if(gpio_config(&gpio_conf) == ESP_OK) ESP_LOGI("Alert configuration","Success");
    else  ESP_LOGI("Alert configuration","Failed");
}

void init_mains_sense(void){
	gpio_config_t gpio_conf ={
		.intr_type = GPIO_INTR_DISABLE,
		.pin_bit_mask = (1ULL<<MAINS_SENSE),
		.mode = GPIO_MODE_INPUT,
		.pull_down_en = GPIO_PULLDOWN_ENABLE,
		.pull_up_en = GPIO_PULLUP_DISABLE,
	};
	if(gpio_config(&gpio_conf) == ESP_OK) ESP_LOGI("Mains configuration","Success");
    else  ESP_LOGI("Mains configuration","Failed");
}

void S4_SW_init(void){
	gpio_pad_select_gpio(S4_SW); /* Select the pin to configure */
	gpio_set_direction(S4_SW,GPIO_MODE_OUTPUT); /* Set the mode of the pin to be configured */
	gpio_set_level(S4_SW,PIN_ON); /* Set output level */
}

void init_led(void){
  for(int i=0;i<3;i++){  
      gpio_pad_select_gpio(led_pins[i]); /* Select the pin to configure */
      gpio_set_direction(led_pins[i], GPIO_MODE_OUTPUT); /* Set the mode of the pin to be configured */
      gpio_set_level(led_pins[i], LED_OFF); /* Set output level */
  }
}

void init_switches(void){
  for(int i=0;i<4;i++){  
      gpio_pad_select_gpio(switch_pins[i]); /* Select the pin to configure */
      gpio_set_direction(switch_pins[i], GPIO_MODE_OUTPUT); /* Set the mode of the pin to be configured */
      gpio_set_level(switch_pins[i], LED_OFF); /* Set output level */
	  vTaskDelay(10/portTICK_PERIOD_MS); /* Cooling time*/
  }	
}
void init_i2c_power(void){
  gpio_pad_select_gpio(I2C_PW_EN); /* Select the pin to configure */
  gpio_set_direction(I2C_PW_EN, GPIO_MODE_OUTPUT); /* Set the mode of the pin to be configured */
  gpio_set_level(I2C_PW_EN, PIN_ON); /* Set output level */
}

void led_control(uint8_t led_color){
  gpio_set_level(LED_RED,  ((led_color)&(0x01))>>0);
  gpio_set_level(LED_GREEN,((led_color)&(0x02))>>1);
  gpio_set_level(LED_BLUE, ((led_color)&(0x04))>>2);
}

void init_buzzer(void){
  gpio_pad_select_gpio(BUZZER); /* Select the pin to configure */
  gpio_set_direction(BUZZER, GPIO_MODE_OUTPUT);  /* Set the mode of the pin to be configured */
  gpio_set_level(BUZZER, BUZZER_OFF); /* Set output level */
  buzzer_state = 0;
}

void buzzer_control(uint8_t buzzer_mode){
  buzzer_state = buzzer_mode;			/* Variable to trace the buzzer status */
  gpio_set_level(BUZZER,buzzer_mode);   /* Set the level of buzzer pin */
}

void toggle_buzzer(void) {
	buzzer_state^=1;
	gpio_set_level(BUZZER, buzzer_state);	
}

uint8_t alertPinSense(void){
	return gpio_get_level(ALERT);
}
/*
 * @Info:   Senses the level of the mains pin
 * @param:  No arguments
 * @Return: 1 if pin high; 0 if pin low
 */

uint8_t mains_sense(void){
  return gpio_get_level(MAINS_SENSE);
}

/*
 * @Info:   Sets the level of battery mosfet
 * @param:  On or Off 
 * @Return: None
 */

void bat_mos_sw(uint8_t status){
  gpio_set_level(BAT_MOS_SW,status);
}

/*
 * @Info:   Sets the level of solar mosfet
 * @param:  On or Off 
 * @Return: None
 */

void sol_mos_sw(uint8_t status){
  gpio_set_level(SOL_MOS_SW,status);
}

/*
 * @Info:   Sets the level of triac
 * @param:  On or Off 
 * @Return: None
 */

void mains_triac_sw(uint8_t status){
  gpio_set_level(TRIAC_SW,status);	
}

/*
 * @Info:   Sets the level of mains relay
 * @param:  On or Off 
 * @Return: None
 */

void mains_rel_sw(uint8_t status){
  gpio_set_level(M_RELAY,status);	
}

/*
 * @Info:   Sets the level of S4 relay
 * @param:  On or Off 
 * @Return: None
 */

void s4_rel_sw(uint8_t status){
  gpio_set_level(S4_SW,status);
}

/*
 * @Info:   Sets the level of battery mosfet to ON
 * @param:  None
 * @Return: None
 */

void bat_mos_sw_on(){
	bat_mos_sw(1);
	switch_state |= BAT_SW;
}

/*
 * @Info:   Sets the level of battery mosfet to OFF
 * @param:  None
 * @Return: None
 */

void bat_mos_sw_off(){
	bat_mos_sw(0);
	switch_state &= ~BAT_SW;
}

/*
 * @Info:   Sets the level of solar mosfet to ON
 * @param:  None
 * @Return: None
 */

void sol_mos_sw_on(){
	sol_mos_sw(1);
	switch_state |= SOL_SW;
}

/*
 * @Info:   Sets the level of solar mosfet to OFF
 * @param:  None
 * @Return: None
 */

void sol_mos_sw_off(){
	sol_mos_sw(0);
	switch_state &= ~SOL_SW;
}

/*
 * @Info:   Sets the level of mains relay to ON
 * @param:  None
 * @Return: None
 */
void mains_rel_sw_on(){
	if(!(switch_state & MAINS_SW)){
		mains_triac_sw(1);
		vTaskDelay(50);
		mains_rel_sw(0);
		vTaskDelay(50);
		mains_triac_sw(0);
		switch_state |= MAINS_SW;
	}
	
}

/*
 * @Info:   Sets the level of mains relay to OFF
 * @param:  None
 * @Return: None
 */
void mains_rel_sw_off(){
	if((switch_state & MAINS_SW)){
		mains_triac_sw(1);
		vTaskDelay(50);
		mains_rel_sw(1);
		vTaskDelay(50);
		mains_triac_sw(0);
		switch_state &= ~MAINS_SW;
	}	
}

/*
 * @Info:  Set the switch state in state change algorithm 
 * @param: switch state; 1,3,4,5,6 or 7
 * @return: None
 */
void turtle_set_sw(uint8_t sw_stat){
	if(sw_stat & MAINS_SW){
		mains_rel_sw_on();
	}
	else{
		mains_rel_sw_off();
	}

	if(sw_stat & BAT_SW){
		bat_mos_sw_on();
	}
	else{
		bat_mos_sw_off();
	}

	if(sw_stat & SOL_SW){
		sol_mos_sw_on();
	}
	else{
		sol_mos_sw_off();
	}
    
#if ICUBE
    if(((algo_param.dev_algo_state == ALGO_STATE_SOL_INV_DIS_7)||(algo_param.dev_algo_state == ALGO_STATE_INV_5))){
        s4_rel_sw(1);
    }
    else{
        s4_rel_sw(0);
    }
#endif
}

/*
 * @Info:   Transit to state 4 then to state 5; 
		    To be used when changing from state1 or state 3 to state 5 
 * @Param:  None
 * @Return: None
 */
void switch_state_4_5(void){
	algo_param.dev_algo_state = ALGO_STATE_BAT_FRC_TRIP_4;
	turtle_set_sw(algo_param.dev_algo_state);
	vTaskDelay(500);
	algo_param.dev_algo_state = ALGO_STATE_INV_5;
	turtle_set_sw(algo_param.dev_algo_state);
}

/*
 * @Info:   Algorithm to blink led;
 * @Param:  None; Uses global variables
 * @Return: None
 */
void led_blink(void){
    blink_time+=1;
    if(blink_time>blink_seconds){
        blink_time = 0;
    }
    if(blink_time<=1){
		led_control(blink_colour);
    }
    else{
		led_control(led_no_color);
    }
}

/*
 * @Info:   Set the global variables which is used in led_blink function
 * @Param:  Led colour and led blink seconds
 * @Return: None
 */
void blink(led_color_t colour,uint16_t seconds){
    blink_colour = colour;
 	blink_seconds = seconds;	
}

/*
 * @Info: Initialization sequence; 
		  1. LED blink; device turn-on buzzer sequence
		  2. Check memory available; Check calibration status
		  3. Turn on state change algorithm
 */
void power_on_initialization(void){
	uint16_t crc_check;
	bat_mos_sw_on();
	sol_mos_sw_off();
	mains_rel_sw_on();
	algo_param.dev_algo_state=ALGO_STATE_INV_5;
	buzzer_control(BUZZER_OFF);
	play_tone2();
	led_control(led_no_color);
	vTaskDelay(100/portTICK_RATE_MS);
	led_control(led_red_color);
	vTaskDelay(500/portTICK_RATE_MS);
	led_control(led_green_color);
	vTaskDelay(500/portTICK_RATE_MS);
	led_control(led_blue_color);
	vTaskDelay(500/portTICK_RATE_MS);
	led_control(led_no_color);
	is_day_flag = NOW_NIGHT_TIME;
	select_i2c_memory(eeprom_addr);
	if(check_memory()==0){
		ESP_LOGI("inside power_on_initialization","Error reading memory");
		post_error(ERR_NO_MEMORY);
	}
	if(!error_flag){
		ESP_LOGI("Power on initialization","Inside parameter initialization");
		load_wifi_param();
		init_algo();
		#if 1
			if(i2c_eeprom_read_int32(EE_CALIB_DONE_ADDR) != CALIB_CONFIG_DONE_VALUE){
				post_error(ERR_NOT_CALIB);
			}
			crc_check = calc_update_eeprom_crc(EE_CALIB_DAT_START_ADDR, CALIB_CONST_LEN, EE_CALIB_CRC_ADDR, 0);
			vTaskDelay(100/portTICK_PERIOD_MS);
			ESP_LOGI("Calibraton check","Completed");
			if(crc_check != i2c_eeprom_read_int16(EE_CALIB_CRC_ADDR)){
				post_error(ERR_NOT_CALIB);
			}
			ESP_LOGI("Calibraton CRC check","Completed");
			if(i2c_eeprom_read_int32(EE_CONFIG_DONE_ADDR) != CALIB_CONFIG_DONE_VALUE){
				post_error(ERR_NOT_CONFIG);
			}
			ESP_LOGI("Configuration check","Completed");
			crc_check = calc_update_eeprom_crc(EE_CONFIG_DAT_START_ADDR, CONFIG_PARAM_LEN, EE_CONFIG_CRC_ADDR, 0);   
			vTaskDelay(100/portTICK_PERIOD_MS);
			if(crc_check != i2c_eeprom_read_int16(EE_CONFIG_CRC_ADDR)){
				post_error(ERR_NOT_CONFIG);
			}
			ESP_LOGI("Configuration CRC heck","Completed");
		#endif	
	}
    show_led();
	ESP_LOGI("Power on initialization","End of initialization");
	// esp_xx_restore();
	// turt_state = TURT_STAT_ON;
	mains_rel_sw_off();
	vTaskDelay(2000/portTICK_RATE_MS);
	mains_rel_sw_on();
	vTaskDelay(500/portTICK_RATE_MS);
	extern bool ledBlinkOn;
	ledBlinkOn=true;
}