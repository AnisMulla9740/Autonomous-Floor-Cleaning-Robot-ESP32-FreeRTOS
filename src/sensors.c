#include "sensors.h"
#include "config.h"
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// External variables
extern QueueHandle_t sensor_queue;

void ultrasonic_init(gpio_num_t trig_pin, gpio_num_t echo_pin) {
    gpio_set_direction(trig_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(echo_pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(echo_pin, GPIO_PULLDOWN_ONLY);
    gpio_set_level(trig_pin, 0);
}

bool get_distance(gpio_num_t trig_pin, gpio_num_t echo_pin, uint16_t *distance) {
    uint32_t timeout = 0;
    
    // Send trigger pulse
    gpio_set_level(trig_pin, 0);
    ets_delay_us(2);
    gpio_set_level(trig_pin, 1);
    ets_delay_us(10);
    gpio_set_level(trig_pin, 0);
    
    // Wait for echo to go high
    timeout = 0;
    while(gpio_get_level(echo_pin) == 0) {
        timeout++;
        if(timeout > 10000) return false;
        ets_delay_us(1);
    }
    uint32_t start_time = esp_timer_get_time();
    
    // Wait for echo to go low
    timeout = 0;
    while(gpio_get_level(echo_pin) == 1) {
        timeout++;
        if(timeout > 60000) return false;
        ets_delay_us(1);
    }
    uint32_t end_time = esp_timer_get_time();
    
    *distance = (end_time - start_time) / 58;
    if(*distance > MAX_DISTANCE) *distance = MAX_DISTANCE;
    
    return true;
}

void sensors_init(void) {
    ultrasonic_init(TRIG_PIN_FRONT, ECHO_PIN_FRONT);
    ultrasonic_init(TRIG_PIN_LEFT, ECHO_PIN_LEFT);
    ultrasonic_init(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT);
}

void sensor_task(void *pvParameters) {
    sensor_data_t sensor_data;
    
    while(1) {
        // Read sensors with error handling
        sensor_data.error_flags = 0;
        
        if(!get_distance(TRIG_PIN_FRONT, ECHO_PIN_FRONT, &sensor_data.front_distance)) {
            sensor_data.error_flags |= 0x01;
        }
        
        if(!get_distance(TRIG_PIN_LEFT, ECHO_PIN_LEFT, &sensor_data.left_distance)) {
            sensor_data.error_flags |= 0x02;
        }
        
        if(!get_distance(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT, &sensor_data.right_distance)) {
            sensor_data.error_flags |= 0x04;
        }
        
        xQueueSend(sensor_queue, &sensor_data, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}