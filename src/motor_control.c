#include "sensors.h"
#include "config.h"
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// External FreeRTOS queue for inter-task communication
extern QueueHandle_t sensor_queue;

/**
 * @brief Initialize ultrasonic sensor GPIO pins
 * 
 * @param trig_pin GPIO used for trigger pulse
 * @param echo_pin GPIO used for echo measurement
 */
void ultrasonic_init(gpio_num_t trig_pin, gpio_num_t echo_pin) {
    // Trigger is OUTPUT
    gpio_set_direction(trig_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(trig_pin, 0);

    // Echo is INPUT with pulldown
    gpio_set_direction(echo_pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(echo_pin, GPIO_PULLDOWN_ONLY);
}

/**
 * @brief Measure distance from ultrasonic sensor
 * 
 * @param trig_pin GPIO trigger pin
 * @param echo_pin GPIO echo pin
 * @param distance Pointer to store result in cm
 * @return true if valid measurement, false if timeout/error
 */
bool get_distance(gpio_num_t trig_pin, gpio_num_t echo_pin, uint16_t *distance) {
    uint32_t timeout = 0;

    // Generate 10us trigger pulse
    gpio_set_level(trig_pin, 0);
    ets_delay_us(2);
    gpio_set_level(trig_pin, 1);
    ets_delay_us(10);
    gpio_set_level(trig_pin, 0);

    // Wait for echo HIGH (start)
    timeout = 0;
    while(gpio_get_level(echo_pin) == 0) {
        if(++timeout > 10000) return false;   // timeout ~10 ms
        ets_delay_us(1);
    }
    uint32_t start_time = esp_timer_get_time();

    // Wait for echo LOW (end)
    timeout = 0;
    while(gpio_get_level(echo_pin) == 1) {
        if(++timeout > 60000) return false;  // timeout ~60 ms
        ets_delay_us(1);
    }
    uint32_t end_time = esp_timer_get_time();

    // Calculate distance (time/58 = cm)
    *distance = (end_time - start_time) / 58;
    
    // Clamp value to maximum limit
    if(*distance > MAX_DISTANCE) {
        *distance = MAX_DISTANCE;
    }

    return true;
}

/**
 * @brief Initialize all ultrasonic sensors
 */
void sensors_init(void) {
    ultrasonic_init(TRIG_PIN_FRONT, ECHO_PIN_FRONT);
    ultrasonic_init(TRIG_PIN_LEFT, ECHO_PIN_LEFT);
    ultrasonic_init(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT);
}

/**
 * @brief FreeRTOS task to read all sensors periodically
 * 
 * This task:
 *  - Reads distances from all sensors
 *  - Sets error_flags if any read fails
 *  - Sends data to sensor_queue
 */
void sensor_task(void *pvParameters) {
    sensor_data_t sensor_data;

    while(1) {
        sensor_data.error_flags = 0;  // reset errors

        // FRONT sensor
        if(!get_distance(TRIG_PIN_FRONT, ECHO_PIN_FRONT, &sensor_data.front_distance)) {
            sensor_data.error_flags |= 0x01;  // set bit0 if error
        }

        // LEFT sensor
        if(!get_distance(TRIG_PIN_LEFT, ECHO_PIN_LEFT, &sensor_data.left_distance)) {
            sensor_data.error_flags |= 0x02;  // set bit1 if error
        }

        // RIGHT sensor
        if(!get_distance(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT, &sensor_data.right_distance)) {
            sensor_data.error_flags |= 0x04;  // set bit2 if error
        }

        // Send results to queue (non-blocking)
        xQueueOverwrite(sensor_queue, &sensor_data);

        // Run every 100 ms
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
