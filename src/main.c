#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include "motor_control.h"
#include "sensors.h"
#include "bluetooth.h"
#include "utilities.h"
#include "config.h"

// Global Variables
QueueHandle_t sensor_queue;
QueueHandle_t command_queue;
SemaphoreHandle_t state_mutex;
robot_state_t current_state;
pid_controller_t pid_left, pid_right;
volatile uint32_t encoder_a_count = 0;
volatile uint32_t encoder_b_count = 0;
float battery_voltage = 0.0;
uint32_t cleaning_time = 0;
float distance_covered = 0.0;

void app_main() {
    // Initialize hardware
    sensors_init();
    motor_control_init();
    encoder_init();
    battery_monitor_init();
    emergency_stop_init();
    status_led_init();
    bluetooth_init();
    
    // Create queues and mutex
    sensor_queue = xQueueCreate(5, sizeof(sensor_data_t));
    command_queue = xQueueCreate(5, 20 * sizeof(char));
    state_mutex = xSemaphoreCreateMutex();
    
    // Initialize state
    current_state = STATE_IDLE;
    
    // Create tasks
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
    xTaskCreate(decision_task, "decision_task", 4096, NULL, 4, NULL);
    xTaskCreate(motor_control_task, "motor_control_task", 4096, NULL, 3, NULL);
    xTaskCreate(battery_task, "battery_task", 2048, NULL, 2, NULL);
    xTaskCreate(bluetooth_task, "bluetooth_task", 4096, NULL, 2, NULL);
    xTaskCreate(path_planning_task, "path_planning_task", 4096, NULL, 3, NULL);
    xTaskCreate(cleaning_task, "cleaning_task", 2048, NULL, 2, NULL);
    xTaskCreate(monitoring_task, "monitoring_task", 4096, NULL, 1, NULL);
    
    // Send startup message
    send_bt_data("ROBOT:READY");
    
    // Start scheduler
    vTaskDelete(NULL);
}