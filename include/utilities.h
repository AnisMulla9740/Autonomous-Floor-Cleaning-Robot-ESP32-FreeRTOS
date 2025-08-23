#ifndef UTILITIES_H
#define UTILITIES_H

#include "config.h"
#include <stdbool.h>

// Function prototypes
void cleaning_motor_control(bool enable, uint8_t speed);
void battery_monitor_init(void);
float read_battery_voltage(void);
void emergency_stop_init(void);
void IRAM_ATTR emergency_stop_isr(void *arg);
void status_led_init(void);
void indicate_state(robot_state_t state);
void path_planning_task(void *pvParameters);
void decision_task(void *pvParameters);
void battery_task(void *pvParameters);
void cleaning_task(void *pvParameters);
void monitoring_task(void *pvParameters);

#endif