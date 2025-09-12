#pragma once

// Motor Driver Pins (TB6612FNG)

// Stby pin
#define STBY 4

// Right motor
#define PWMA 5
#define AIN1 7
#define AIN2 6
// Left motor
#define PWMB 10
#define BIN1 8
#define BIN2 9

// Motor PWM Range
#define MAX_PWM 100
#define MIN_PWM -100

// Encoder Pins

// Left encoder
#define ENCODER0_PINA 2
#define ENCODER0_PINB 3
// Right encoder
#define ENCODER1_PINA 1
#define ENCODER1_PINB 0

// Encoder Count Per Revolution
#define ENCODER_CPR 1320  // 30 (gear ratio) x 11 (magnetic loops/encoder lines) x 4 (quadrature edges)

// PID Control Parameters - Individual for each wheel
#define LEFT_PID_KP 5.0f
#define LEFT_PID_KI 20.0f
#define LEFT_PID_KD 0.0f

#define RIGHT_PID_KP 5.0f   // Different values for right wheel
#define RIGHT_PID_KI 20.0f
#define RIGHT_PID_KD 0.0f

#define PID_MAX_OUTPUT MAX_PWM
#define PID_MIN_OUTPUT MIN_PWM

extern float left_target_speed;
extern float right_target_speed;

// Robot Dimensions
#define WHEEL_RADIUS 0.034  // in meters
#define WHEEL_SEPARATION 0.194  // in meters

// Control Timing
#define CMD_VEL_TIMEOUT_MS 500  // ms before stopping motors if no cmd_vel received

// Define ROS_DOMAIN_ID
#define ROS_DOMAIN_ID 47    // Must be same as computer to communicate

