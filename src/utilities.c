#include "utilities.h"
#include "config.h"
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <stdio.h>

// External variables
extern QueueHandle_t sensor_queue;
extern SemaphoreHandle_t state_mutex;
extern robot_state_t current_state;
extern float battery_voltage;
extern uint32_t cleaning_time;
extern float distance_covered;

void cleaning_motor_control(bool enable, uint8_t speed) {
    static ledc_channel_config_t cleaning_channel;
    
    if(enable) {
        cleaning_channel.gpio_num = CLEANING_MOTOR_PIN;
        cleaning_channel.speed_mode = LEDC_LOW_SPEED_MODE;
        cleaning_channel.channel = LEDC_CHANNEL_2;
        cleaning_channel.timer_sel = LEDC_TIMER_0;
        cleaning_channel.duty = (speed * 1023) / 100;
        cleaning_channel.hpoint = 0;
        ledc_channel_config(&cleaning_channel);
    } else {
        ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);
    }
}

void battery_monitor_init(void) {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(BATTERY_ADC_PIN, ADC_ATTEN_DB_11);
}

float read_battery_voltage(void) {
    int adc_reading = 0;
    for(int i = 0; i < 10; i++) {
        adc_reading += adc1_get_raw(BATTERY_ADC_PIN);
        ets_delay_us(100);
    }
    adc_reading /= 10;
    
    // Convert ADC reading to voltage
    float voltage = (adc_reading * 3.3 * 2) / 4095.0;
    return voltage;
}

void emergency_stop_init(void) {
    gpio_set_direction(EMERGENCY_STOP_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(EMERGENCY_STOP_PIN, GPIO_PULLUP_ONLY);
    gpio_set_intr_type(EMERGENCY_STOP_PIN, GPIO_INTR_NEGEDGE);
    gpio_isr_handler_add(EMERGENCY_STOP_PIN, emergency_stop_isr, NULL);
}

void IRAM_ATTR emergency_stop_isr(void *arg) {
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    current_state = STATE_EMERGENCY_STOP;
    xSemaphoreGive(state_mutex);
}

void status_led_init(void) {
    gpio_set_direction(STATUS_LED_PIN, GPIO_MODE_OUTPUT);
}

void indicate_state(robot_state_t state) {
    static uint64_t last_blink = 0;
    uint64_t current_time = esp_timer_get_time();
    
    switch(state) {
        case STATE_IDLE:
            gpio_set_level(STATUS_LED_PIN, 0);
            break;
        case STATE_CLEANING:
            if(current_time - last_blink > 500000) {
                gpio_set_level(STATUS_LED_PIN, !gpio_get_level(STATUS_LED_PIN));
                last_blink = current_time;
            }
            break;
        case STATE_OBSTACLE_AVOIDANCE:
            if(current_time - last_blink > 200000) {
                gpio_set_level(STATUS_LED_PIN, !gpio_get_level(STATUS_LED_PIN));
                last_blink = current_time;
            }
            break;
        case STATE_EMERGENCY_STOP:
            gpio_set_level(STATUS_LED_PIN, 1);
            break;
        case STATE_LOW_BATTERY:
            if(current_time - last_blink > 1000000) {
                gpio_set_level(STATUS_LED_PIN, 1);
                ets_delay_us(100000);
                gpio_set_level(STATUS_LED_PIN, 0);
                ets_delay_us(100000);
                gpio_set_level(STATUS_LED_PIN, 1);
                ets_delay_us(100000);
                gpio_set_level(STATUS_LED_PIN, 0);
                last_blink = current_time;
            }
            break;
        default:
            break;
    }
}

void path_planning_task(void *pvParameters) {
    uint32_t spiral_step = 0;
    uint32_t last_direction_change = 0;
    
    while(1) {
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        
        if(current_state == STATE_CLEANING) {
            uint64_t current_time = esp_timer_get_time();
            
            if(current_time - last_direction_change > 5000000) {
                spiral_step = (spiral_step + 1) % 4;
                last_direction_change = current_time;
                
                switch(spiral_step) {
                    case 0:
                        set_movement(MOVEMENT_FORWARD);
                        break;
                    case 1:
                        set_movement(MOVEMENT_RIGHT);
                        vTaskDelay(pdMS_TO_TICKS(500));
                        set_movement(MOVEMENT_FORWARD);
                        break;
                    case 2:
                        set_movement(MOVEMENT_RIGHT);
                        vTaskDelay(pdMS_TO_TICKS(500));
                        set_movement(MOVEMENT_FORWARD);
                        break;
                    case 3:
                        set_movement(MOVEMENT_BACKWARD);
                        vTaskDelay(pdMS_TO_TICKS(2000));
                        set_movement(MOVEMENT_RIGHT);
                        vTaskDelay(pdMS_TO_TICKS(1000));
                        spiral_step = 0;
                        break;
                }
            }
        }
        
        xSemaphoreGive(state_mutex);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void decision_task(void *pvParameters) {
    sensor_data_t sensor_data;
    movement_state_t current_movement = MOVEMENT_STOP;
    
    while(1) {
        if(xQueueReceive(sensor_queue, &sensor_data, portMAX_DELAY)) {
            xSemaphoreTake(state_mutex, portMAX_DELAY);
            
            if(current_state == STATE_EMERGENCY_STOP) {
                set_movement(MOVEMENT_STOP);
                cleaning_motor_control(false, 0);
                xSemaphoreGive(state_mutex);
                continue;
            }
            
            if(current_state == STATE_LOW_BATTERY) {
                set_movement(MOVEMENT_FORWARD);
                xSemaphoreGive(state_mutex);
                continue;
            }
            
            // Obstacle avoidance logic
            if(sensor_data.front_distance < MIN_DISTANCE) {
                current_state = STATE_OBSTACLE_AVOIDANCE;
                if(sensor_data.right_distance > sensor_data.left_distance) {
                    current_movement = MOVEMENT_RIGHT;
                } else {
                    current_movement = MOVEMENT_LEFT;
                }
                vTaskDelay(pdMS_TO_TICKS(500));
            } else if(sensor_data.left_distance < MIN_DISTANCE/2) {
                current_state = STATE_OBSTACLE_AVOIDANCE;
                current_movement = MOVEMENT_RIGHT;
                vTaskDelay(pdMS_TO_TICKS(200));
            } else if(sensor_data.right_distance < MIN_DISTANCE/2) {
                current_state = STATE_OBSTACLE_AVOIDANCE;
                current_movement = MOVEMENT_LEFT;
                vTaskDelay(pdMS_TO_TICKS(200));
            } else {
                if(current_state == STATE_OBSTACLE_AVOIDANCE) {
                    current_state = STATE_CLEANING;
                }
                current_movement = MOVEMENT_FORWARD;
            }
            
            set_movement(current_movement);
            xSemaphoreGive(state_mutex);
        }
    }
}

void battery_task(void *pvParameters) {
    while(1) {
        battery_voltage = read_battery_voltage();
        
        if(battery_voltage < LOW_BATTERY_THRESHOLD) {
            xSemaphoreTake(state_mutex, portMAX_DELAY);
            if(current_state != STATE_EMERGENCY_STOP) {
                current_state = STATE_LOW_BATTERY;
                char alert[50];
                snprintf(alert, sizeof(alert), "ALERT:LOW_BATTERY:%.2fV", battery_voltage);
                send_bt_data(alert);
            }
            xSemaphoreGive(state_mutex);
        }
        
        vTaskDelay(pdMS_TO_TICKS(BATTERY_CHECK_INTERVAL));
    }
}

void cleaning_task(void *pvParameters) {
    while(1) {
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        
        if(current_state == STATE_CLEANING) {
            cleaning_motor_control(true, 80);
            cleaning_time += 100;
        } else if(current_state == STATE_OBSTACLE_AVOIDANCE) {
            cleaning_motor_control(true, 50);
        } else {
            cleaning_motor_control(false, 0);
        }
        
        xSemaphoreGive(state_mutex);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void monitoring_task(void *pvParameters) {
    char status_msg[100];
    
    while(1) {
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        
        snprintf(status_msg, sizeof(status_msg), 
                 "STATUS:%d,BATT:%.2fV,TIME:%ds,DIST:%.2fcm", 
                 current_state, battery_voltage, cleaning_time/1000, distance_covered);
        
        send_bt_data(status_msg);
        
        indicate_state(current_state);
        
        xSemaphoreGive(state_mutex);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}