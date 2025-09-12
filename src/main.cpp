#include <Arduino.h>
#include "config.h"
#include "encoder.h"
#include "motor_control.h"
#include "pid_controller.h"
#include "microros.h"

// PID controllers for left and right motors
PIDController left_pid(LEFT_PID_KP, LEFT_PID_KI, LEFT_PID_KD, PID_MAX_OUTPUT, PID_MIN_OUTPUT);
PIDController right_pid(RIGHT_PID_KP, RIGHT_PID_KI, RIGHT_PID_KD, PID_MAX_OUTPUT, PID_MIN_OUTPUT);

// Variables for target speeds
float left_target_speed = 0.0f;
float right_target_speed = 0.0f;

// Timing and encoder tracking variables
unsigned long last_control_time = 0;
unsigned long last_left_count = 0;
unsigned long last_right_count = 0;
const unsigned long CONTROL_PERIOD_MS = 10; // 10ms control loop

void setup() {
    Serial.begin(115200);
    // Initialize hardware
    setup_motors();
    setup_encoders();
    
    // Initialize micro-ROS
    setup_microros();
    
    // Initialize encoder tracking
    last_left_count = left_encoder_count;
    last_right_count = right_encoder_count;
    last_control_time = millis();
}

void loop() {
    // Handle micro-ROS communication
    microros_loop();
    
    // Run control loop at fixed interval
    unsigned long current_time = millis();
    if (current_time - last_control_time >= CONTROL_PERIOD_MS) {
        float delta_time = (current_time - last_control_time) / 1000.0f;
        
        // Calculate encoder deltas
        int delta_left = left_encoder_count - last_left_count;
        int delta_right = right_encoder_count - last_right_count;
        
        // Update tracking variables
        last_control_time = current_time;
        last_left_count = left_encoder_count;
        last_right_count = right_encoder_count;
        
        // Calculate current wheel speeds in rad/s
        float left_speed = calculate_speed(delta_left, delta_time);
        float right_speed = calculate_speed(delta_right, delta_time);
        
        // Publish wheel speeds for monitoring
        publish_wheel_speeds(left_speed, right_speed);
        
        // Check for cmd_vel timeout
        if (cmd_vel_timeout_exceeded()) {
            left_target_speed = 0.0f;
            right_target_speed = 0.0f;
        }
        
        // Compute PID outputs
        int left_pwm = (int)left_pid.compute(left_target_speed, left_speed);
        int right_pwm = (int)right_pid.compute(right_target_speed, right_speed);
        
        // Apply motor control
        set_motor_speed(PWMA, AIN1, AIN2, right_pwm);
        set_motor_speed(PWMB, BIN1, BIN2, left_pwm);
    }
}