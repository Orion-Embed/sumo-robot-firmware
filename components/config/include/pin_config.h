#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H
#include "driver/gpio.h"

// BTS7960 Motor Driver 1 (Left Side Motors)
#define BTS1_RPWM GPIO_NUM_18 // Right PWM for Motor 1 & 2
#define BTS1_LPWM GPIO_NUM_19 // Left PWM for Motor 1 & 2
#define BTS1_R_EN GPIO_NUM_21 // Right Enable
#define BTS1_L_EN GPIO_NUM_22 // Left Enable

// BTS7960 Motor Driver 2 (Right Side Motors)
#define BTS2_RPWM GPIO_NUM_5  // Right PWM for Motor 3 & 4
#define BTS2_LPWM GPIO_NUM_17 // Left PWM for Motor 3 & 4
#define BTS2_R_EN GPIO_NUM_16 // Right Enable
#define BTS2_L_EN GPIO_NUM_4  // Left Enable

// Motor Assignments (4WD Robot)
#define MOTOR_FRONT_LEFT 0  // Connected to BTS7960 #1
#define MOTOR_REAR_LEFT 1   // Connected to BTS7960 #1
#define MOTOR_FRONT_RIGHT 2 // Connected to BTS7960 #2
#define MOTOR_REAR_RIGHT 3  // Connected to BTS7960 #2

// IR Sensor Pins for Ring Boundary Detection
#define IR_FRONT_LEFT GPIO_NUM_13
#define IR_FRONT_RIGHT GPIO_NUM_12
#define IR_REAR_LEFT GPIO_NUM_14
#define IR_REAR_RIGHT GPIO_NUM_27

// Ultrasonic Sensor Pins for Enemy Detection
#define US_FRONT_TRIG GPIO_NUM_32
#define US_FRONT_ECHO GPIO_NUM_35

#define US_REAR_TRIG GPIO_NUM_33
#define US_REAR_ECHO GPIO_NUM_34

// PWM Configuration for BTS7960
#define BTS7960_PWM_FREQ 10000 // 10kHz PWM frequency
#define BTS7960_PWM_RES LEDC_TIMER_8_BIT
#define BTS7960_PWM_CH_RPWM1 LEDC_CHANNEL_0
#define BTS7960_PWM_CH_LPWM1 LEDC_CHANNEL_1
#define BTS7960_PWM_CH_RPWM2 LEDC_CHANNEL_2
#define BTS7960_PWM_CH_LPWM2 LEDC_CHANNEL_3

#endif
