#ifndef _esp32s2_main_h
#define _esp32s2_main_h

#include "stdio.h"
#include "stdlib.h"
#include "stdint.h"

#define ICUBE 0

typedef enum{
    no_press = 0,
    single_press = 1,
    double_press = 2,
    long_press   = 3,
}switchpress_value;

void init_spiffs(void);
void read_domain(char *ptr);
void read_spiffs( char *ptr, int type );
void update_spiffs( char *param, int type );
void fcn_factory_reset(void);

#endif