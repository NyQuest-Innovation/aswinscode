#ifndef _esp32s2_gpio_h_
#define _esp32s2_gpio_h_

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_types.h"
#include "esp32s2_socket.h"

#define Bit(x) (1UL << (x))

#define LED_RED   20
#define LED_GREEN 21
#define LED_BLUE  26

#if 0
    #warning "Should be changed to 35"
    #define BUZZER    14
#else
    #define BUZZER    35
#endif

#define ALERT 36
#define BAT_MOS_SW    41

#if 0
    #warning "Should be changed to 45"
    #define SOL_MOS_SW    39
#else
    #define SOL_MOS_SW    45
#endif

#define M_RELAY   40
#define TRIAC_SW  42

#define S4_SW     13
#define MAINS_SENSE 12 //6th pin is ACV and it is not used for ACSENSE. Alert pin of the schematic is used for ACSENSE

#define I2C_PW_EN 13

#define LED_ON  1
#define LED_OFF 0

#define PIN_ON  1
#define PIN_OFF 0

#define BUZZER_ON 1
#define BUZZER_OFF 0

typedef enum{
    led_no_color = 0x0,    
    led_red_color = 0x1,
    led_green_color = 0x2,
    led_blue_color = 0x4,
    led_yellow_color = led_red_color+led_green_color,
    led_magenta_color = led_red_color+led_blue_color,
    led_cyan_color = led_green_color+led_blue_color,
    led_white_color = led_red_color+led_cyan_color,
} led_color_t;

typedef enum{
    MAINS_SW = Bit(0),
    SOL_SW   = Bit(1),
    BAT_SW   = Bit(2),
}switch_state_value_t;

uint16_t blink_time,blink_seconds;
led_color_t blink_colour;
typedef enum{
    ONE_SECOND_BLINK = 10,
    FIVE_SECOND_BLINK = 50,
    HALF_SECOND_BLINK = 3,
    FULL_ON = 0,
}led_blink_time;

extern uint8_t switch_state;
extern void init_mains_sense(void);
extern void init_switches(void);
extern void turtle_set_sw(uint8_t sw_stat);
extern void beep(uint8_t beep_count, uint16_t beep_dur);
extern void led_on(uint16_t led_name);
extern void led_off(uint16_t led_name);
extern void led_toggle(uint16_t led_name);
extern void sw_intr_en();
extern void sw_intr_dis();
extern uint8_t is_bt_connected();
extern void bt_conn_intr_en();
extern void bt_conn_intr_dis();
extern void clear_all();
extern void led_off_all();
extern void led_on_all();
// extern void led_blink(uint16_t led_name);
extern void led_blinker_en(uint16_t led_col, uint16_t led_on_tim, uint16_t led_off_tim);
extern void led_blinker_dis(uint16_t led_col);
extern uint8_t get_blinker_stat();
extern void led_blinker_action();
extern void led_device_state_en(uint16_t led_col, uint16_t led_on_tim, uint16_t led_off_tim);
extern void led_show_device_state();
extern void led_device_state_dis(uint16_t led_col);
extern void test_portb_intr();
extern void test_bt_conn_intr();
extern void switch_state_4_7();
extern void WDT_Fcn();
extern void init_led(void);
extern void init_i2c_power(void);
extern void led_control(uint8_t led_color);
extern void init_buzzer(void);
extern void buzzer_control(uint8_t buzzer_mode);
extern void toggle_buzzer(void);
extern uint8_t mains_sense(void);
extern void bat_mos_sw(uint8_t status);
extern void sol_mos_sw(uint8_t status);
extern void mains_triac_sw(uint8_t status);
extern void mains_rel_sw(uint8_t status);
extern void s4_rel_sw(uint8_t status);
extern void bat_mos_sw_on();
extern void bat_mos_sw_off();
extern void sol_mos_sw_on();
extern void sol_mos_sw_off();
extern void mains_rel_sw_on();
extern void mains_rel_sw_off();
extern void turtle_set_sw(uint8_t sw_stat);
extern void switch_state_4_5(void);
extern void S4_SW_init(void);
extern void led_blink(void);
extern void blink(led_color_t colour,uint16_t seconds);
extern void power_on_initialization();
extern void initializeBatteryOvervoltageSense(void);
extern uint8_t alertPinSense(void);
extern void enableALERTInterrupt();
#endif