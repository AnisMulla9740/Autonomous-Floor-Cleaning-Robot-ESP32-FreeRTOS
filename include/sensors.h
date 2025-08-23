#ifndef SENSORS_H
#define SENSORS_H

#include "config.h"
#include <driver/gpio.h>
#include <stdbool.h>

// Function prototypes
void ultrasonic_init(gpio_num_t trig_pin, gpio_num_t echo_pin);
bool get_distance(gpio_num_t trig_pin, gpio_num_t echo_pin, uint16_t *distance);
void sensors_init(void);
void sensor_task(void *pvParameters);

#endif