#ifndef CONFIG_H
#define CONFIG_H

#include <driver/gpio.h>
#include <driver/adc.h>

// Cytron MD10C Motor Driver Pin Definitions
#define MOTOR_A_PWM        GPIO_NUM_16
#define MOTOR_A_DIR        GPIO_NUM_17
#define MOTOR_B_PWM        GPIO_NUM_18
#define MOTOR_B_DIR        GPIO_NUM_19

// Ultrasonic Sensor Pins
#define TRIG_PIN_LEFT      GPIO_NUM_13
#define ECHO_PIN_LEFT      GPIO_NUM_12
#define TRIG_PIN_RIGHT     GPIO_NUM_14
#define ECHO_PIN_RIGHT     GPIO_NUM_27
#define TRIG_PIN_FRONT     GPIO_NUM_26
#define ECHO_PIN_FRONT     GPIO_NUM_25

// Encoder Pins
#define ENCODER_A_PIN      GPIO_NUM_34
#define ENCODER_B_PIN      GPIO_NUM_35

// Cleaning Mechanism and Other Pins
#define CLEANING_MOTOR_PIN GPIO_NUM_23
#define BATTERY_ADC_PIN    ADC1_CHANNEL_0  // GPIO36
#define EMERGENCY_STOP_PIN GPIO_NUM_4
#define STATUS_LED_PIN     GPIO_NUM_2      // Built-in LED

// Constants
#define MAX_DISTANCE 200
#define MIN_DISTANCE 15
#define PWM_FREQ     5000
#define BATTERY_CHECK_INTERVAL 60000
#define LOW_BATTERY_THRESHOLD 3.3
#define ENCODER_PPR 20
#define WHEEL_CIRCUMFERENCE 31.4

// PID Constants
#define KP 0.8
#define KI 0.2
#define KD 0.1

// Robot States
typedef enum {
    STATE_IDLE,
    STATE_CLEANING,
    STATE_OBSTACLE_AVOIDANCE,
    STATE_RETURN_TO_DOCK,
    STATE_EMERGENCY_STOP,
    STATE_LOW_BATTERY
} robot_state_t;

// Sensor Data Structure
typedef struct {
    uint16_t front_distance;
    uint16_t left_distance;
    uint16_t right_distance;
    uint8_t error_flags;
} sensor_data_t;

// Movement States
typedef enum {
    MOVEMENT_FORWARD,
    MOVEMENT_BACKWARD,
    MOVEMENT_LEFT,
    MOVEMENT_RIGHT,
    MOVEMENT_STOP
} movement_state_t;

// PID Controller Structure
typedef struct {
    float kp, ki, kd;
    float setpoint;
    float integral;
    float previous_error;
} pid_controller_t;

#endif