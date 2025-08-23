#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include "config.h"

// Function prototypes
void bluetooth_init(void);
void send_bt_data(const char *data);
void bluetooth_task(void *pvParameters);

#endif