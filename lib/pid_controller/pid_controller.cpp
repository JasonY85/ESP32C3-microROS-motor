#include "pid_controller.h"
#include <Arduino.h>

PIDController::PIDController(float kp, float ki, float kd, float max_output, float min_output)
    : kp_(kp), ki_(ki), kd_(kd), max_output_(max_output), min_output_(min_output),
      integral_(0), previous_error_(0), last_time_(0) {}

float PIDController::compute(float setpoint, float input) {
    unsigned long now = millis();
    float dt = (now - last_time_) / 1000.0f;
    if (dt <= 0) dt = 0.01f; // Default to 10ms if time hasn't changed
    
    float error = setpoint - input;
    
    // Proportional term
    float p_term = kp_ * error;
    
    // Integral term with anti-windup
    integral_ += error * dt;
    if (ki_ != 0) {
        integral_ = constrain(integral_, min_output_ / ki_, max_output_ / ki_);
    }
    float i_term = ki_ * integral_;
    
    // Derivative term
    float d_term = 0;
    if (dt > 0) {
        d_term = kd_ * (error - previous_error_) / dt;
    }
    
    // Calculate total output
    float output = p_term + i_term + d_term;
    output = constrain(output, min_output_, max_output_);
    
    // Save state for next iteration
    previous_error_ = error;
    last_time_ = now;
    
    return output;
}

void PIDController::reset() {
    integral_ = 0;
    previous_error_ = 0;
    last_time_ = millis();
}