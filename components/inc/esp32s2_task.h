#ifndef _esp32s2_tasks_h
#define _esp32s2_tasks_h

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#define overvoltageProtectionTaskEvent  1
#define stateMachineTaskEvent           2
#define dataLoggingTaskEvent            4
#define allEvents                       (overvoltageProtectionTaskEvent|stateMachineTaskEvent|dataLoggingTaskEvent)

void createSemaphores(void);
void test_wifi_config(void *param);
void OnConnected(void *para);
void on_10m_second_timer(void * param);
void on_100m_second_timer(void * param);
void on_1_second_timer(void * param);
void on_15_seconds_timer(void * param);
void init_data(void);
void peripheral_initializaton(void);
void least_priority_operations(void);
void main_function(void *para);
void createTasks(void);
void watchdogTask(void);

#endif