#pragma once

#include <Arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32.h>

typedef enum {
    MICROROS_DISCONNECTED,
    MICROROS_CONNECTED,
    MICROROS_CONNECTION_LOST
} microROS_connection_state_t;

microROS_connection_state_t get_microROS_connection_state();

void setup_microros();
void microros_loop();
bool is_microros_connected();
void publish_wheel_speeds(float left_speed, float right_speed);
void reset_cmd_vel_timeout();
bool cmd_vel_timeout_exceeded();

extern unsigned long last_cmd_vel_time;
extern float left_target_speed;
extern float right_target_speed;