#pragma once

//Function declaration
void setup_joystick();
void joystick_loop();

extern unsigned long last_cmd_vel_time;
extern float linear_vel;
extern float angular_vel;