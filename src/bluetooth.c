#include "bluetooth.h"
#include "config.h"
#include <esp_bt.h>
#include <esp_bt_main.h>
#include <esp_gap_bt_api.h>
#include <esp_spp_api.h>
#include <string.h>
#include <stdio.h>

// External variables
extern QueueHandle_t command_queue;
extern SemaphoreHandle_t state_mutex;
extern robot_state_t current_state;

// Bluetooth callbacks
void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
    if(event == ESP_SPP_DATA_IND_EVT) {
        char *data = (char *)param->data_ind.data;
        xQueueSend(command_queue, data, 0);
    }
}

void bluetooth_init(void) {
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    esp_bluedroid_init();
    esp_bluedroid_enable();
    esp_spp_register_callback(esp_spp_cb);
    esp_spp_init(ESP_SPP_MODE_CB);
}

void send_bt_data(const char *data) {
    printf("BT: %s\n", data);
}

void bluetooth_task(void *pvParameters) {
    char command[20];
    
    while(1) {
        if(xQueueReceive(command_queue, &command, portMAX_DELAY)) {
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
                // Send current status
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