#ifndef _esp32s2_button_h
#define _esp32s2_button_h

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_types.h"
#include "esp32s2_socket.h"
#include "esp32s2_task.h"

#if 0
    #warning "Should be changed to 46"
    #define BUTTON    35
#else
    #define BUTTON    46
#endif

void init_pushbutton(void);
uint8_t button_routine(void);
void buttonPressDetection_routine(void *para);
void singlePressOperation(void);
void doublePressOperation(void);
void longPressOperation(void);
void buttonPressTask(void *para);

#endif