#include "motor_control.h"
#include "config.h"
#include <driver/ledc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// External variables
extern volatile uint32_t encoder_a_count;
extern volatile uint32_t encoder_b_count;
extern pid_controller_t pid_left, pid_right;

void motor_control_init(void) {
    // Configure motor control pins for Cytron MD10C
    gpio_set_direction(MOTOR_A_DIR, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_B_DIR, GPIO_MODE_OUTPUT);

    // Configure PWM for Cytron MD10C
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQ,
    };
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t channel_conf = {
        .gpio_num = MOTOR_A_PWM,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0
    };
    ledc_channel_config(&channel_conf);

    channel_conf.gpio_num = MOTOR_B_PWM;
    channel_conf.channel = LEDC_CHANNEL_1;
    ledc_channel_config(&channel_conf);
}

void set_motor_speed(uint8_t duty_cycle) {
    duty_cycle = duty_cycle > 100 ? 100 : duty_cycle;
    uint32_t duty = (duty_cycle * 1023) / 100;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

void set_movement(movement_state_t state) {
    switch(state) {
        case MOVEMENT_FORWARD:
            gpio_set_level(MOTOR_A_DIR, 1);
            gpio_set_level(MOTOR_B_DIR, 1);
            break;
        case MOVEMENT_BACKWARD:
            gpio_set_level(MOTOR_A_DIR, 0);
            gpio_set_level(MOTOR_B_DIR, 0);
            break;
        case MOVEMENT_LEFT:
            gpio_set_level(MOTOR_A_DIR, 0);
            gpio_set_level(MOTOR_B_DIR, 1);
            break;
        case MOVEMENT_RIGHT:
            gpio_set_level(MOTOR_A_DIR, 1);
            gpio_set_level(MOTOR_B_DIR, 0);
            break;
        case MOVEMENT_STOP:
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
            break;
    }
}

void encoder_init(void) {
    gpio_set_direction(ENCODER_A_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(ENCODER_B_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(ENCODER_A_PIN, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(ENCODER_B_PIN, GPIO_PULLUP_ONLY);
    
    // Install ISR service
    gpio_install_isr_service(0);
    
    // Configure interrupts for encoder pins
    gpio_set_intr_type(ENCODER_A_PIN, GPIO_INTR_POSEDGE);
    gpio_set_intr_type(ENCODER_B_PIN, GPIO_INTR_POSEDGE);
    
    // Add ISR handlers
    gpio_isr_handler_add(ENCODER_A_PIN, encoder_a_isr, NULL);
    gpio_isr_handler_add(ENCODER_B_PIN, encoder_b_isr, NULL);
}

void IRAM_ATTR encoder_a_isr(void *arg) {
    encoder_a_count++;
}

void IRAM_ATTR encoder_b_isr(void *arg) {
    encoder_b_count++;
}

void calculate_speed(float *left_speed, float *right_speed) {
    static uint32_t last_encoder_a = 0;
    static uint32_t last_encoder_b = 0;
    static uint64_t last_time = 0;
    
    uint64_t current_time = esp_timer_get_time();
    uint32_t elapsed_time = current_time - last_time;
    
    if(elapsed_time > 100000) {
        *left_speed = (encoder_a_count - last_encoder_a) * WHEEL_CIRCUMFERENCE / 
                     (ENCODER_PPR * (elapsed_time / 1000000.0));
        *right_speed = (encoder_b_count - last_encoder_b) * WHEEL_CIRCUMFERENCE / 
                      (ENCODER_PPR * (elapsed_time / 1000000.0));
        
        last_encoder_a = encoder_a_count;
        last_encoder_b = encoder_b_count;
        last_time = current_time;
    }
}

void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float setpoint) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->setpoint = setpoint;
    pid->integral = 0;
    pid->previous_error = 0;
}

float pid_update(pid_controller_t *pid, float current_value) {
    float error = pid->setpoint - current_value;
    pid->integral += error;
    float derivative = error - pid->previous_error;
    pid->previous_error = error;
    
    return (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
}

void motor_control_task(void *pvParameters) {
    float left_speed = 0, right_speed = 0;
    float left_output = 0, right_output = 0;
    
    pid_init(&pid_left, KP, KI, KD, 30.0);
    pid_init(&pid_right, KP, KI, KD, 30.0);
    
    while(1) {
        calculate_speed(&left_speed, &right_speed);
        
        left_output = pid_update(&pid_left, left_speed);
        right_output = pid_update(&pid_right, right_speed);
        
        set_motor_speed(70 + (uint8_t)left_output);
        
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}