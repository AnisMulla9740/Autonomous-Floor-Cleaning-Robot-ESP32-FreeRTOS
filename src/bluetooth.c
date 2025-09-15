#include "bluetooth.h"
#include "config.h"
#include <esp_bt.h>
#include <esp_bt_main.h>
#include <esp_gap_bt_api.h>
#include <esp_spp_api.h>
#include <string.h>
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

// External shared objects
extern QueueHandle_t command_queue;
extern SemaphoreHandle_t state_mutex;
extern robot_state_t current_state;

// Store active client handle (set on connection)
static uint32_t spp_client_handle = 0;

/**
 * @brief Bluetooth SPP callback handler
 */
void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
    switch(event) {
        case ESP_SPP_SRV_OPEN_EVT: // client connected
            spp_client_handle = param->srv_open.handle;
            send_bt_data("ROBOT:READY");
            break;

        case ESP_SPP_CLOSE_EVT: // client disconnected
            spp_client_handle = 0;
            break;

        case ESP_SPP_DATA_IND_EVT: { // data received
            char cmd_buffer[20];
            int len = param->data_ind.len;

            // Copy safely with null-termination
            if(len >= sizeof(cmd_buffer)) len = sizeof(cmd_buffer) - 1;
            memcpy(cmd_buffer, param->data_ind.data, len);
            cmd_buffer[len] = '\0';

            // Send to queue (non-blocking)
            xQueueSend(command_queue, cmd_buffer, 0);
            break;
        }

        default:
            break;
    }
}

/**
 * @brief Initialize Bluetooth SPP server
 */
void bluetooth_init(void) {
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    if(esp_bt_controller_init(&bt_cfg) != ESP_OK) {
        printf("BT Controller init failed\n");
        return;
    }
    if(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT) != ESP_OK) {
        printf("BT Controller enable failed\n");
        return;
    }
    if(esp_bluedroid_init() != ESP_OK || esp_bluedroid_enable() != ESP_OK) {
        printf("Bluedroid init/enable failed\n");
        return;
    }

    esp_spp_register_callback(esp_spp_cb);
    esp_spp_init(ESP_SPP_MODE_CB);

    printf("Bluetooth SPP initialized\n");
}

/**
 * @brief Send data over Bluetooth
 */
void send_bt_data(const char *data) {
    if (spp_client_handle != 0) { // only if client connected
        esp_spp_write(spp_client_handle, strlen(data), (uint8_t *)data);
    }
    printf("BT: %s\n", data); // also log locally
}

/**
 * @brief FreeRTOS task: process commands received via BT
 */
void bluetooth_task(void *pvParameters) {
    char command[20];

    while(1) {
        // Block until new command received
        if(xQueueReceive(command_queue, command, portMAX_DELAY)) {
            xSemaphoreTake(state_mutex, portMAX_DELAY);

            if(strcmp(command, "START") == 0) {
                current_state = STATE_CLEANING;
                send_bt_data("CMD:START_OK");

            } else if(strcmp(command, "STOP") == 0) {
                current_state = STATE_IDLE;
                send_bt_data("CMD:STOP_OK");

            } else if(strcmp(command, "DOCK") == 0) {
                current_state = STATE_RETURN_TO_DOCK;
                send_bt_data("CMD:DOCK_OK");

            } else if(strcmp(command, "EMERGENCY_STOP") == 0) {
                current_state = STATE_EMERGENCY_STOP;
                send_bt_data("CMD:EMERGENCY_STOP_OK");

            } else if(strcmp(command, "STATUS") == 0) {
                // Report system status
                char status_msg[100];
                extern float battery_voltage;
                extern uint32_t cleaning_time;
                extern float distance_covered;

                snprintf(status_msg, sizeof(status_msg), 
                         "STATUS:%d,BATT:%.2fV,TIME:%ds,DIST:%.2fcm", 
                         current_state, battery_voltage, cleaning_time/1000, distance_covered);
                send_bt_data(status_msg);
            }

            xSemaphoreGive(state_mutex);
        }
    }
}
