#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "config.h"
#include <stdint.h>
#include <stdbool.h>

// Function prototypes
void motor_control_init(void);
void set_motor_speed(uint8_t duty_cycle);
void set_movement(movement_state_t state);
void encoder_init(void);
void IRAM_ATTR encoder_a_isr(void *arg);
void IRAM_ATTR encoder_b_isr(void *arg);
void calculate_speed(float *left_speed, float *right_speed);
void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float setpoint);
float pid_update(pid_controller_t *pid, float current_value);
void motor_control_task(void *pvParameters);

#endif