#include <Arduino.h>
#include "joystick.h"
#include "config.h"
/*
   Gamepad module provides three different mode namely Digital, JoyStick and Accerleometer.

   You can reduce the size of library compiled by enabling only those modules that you want to
   use. For this first define CUSTOM_SETTINGS followed by defining INCLUDE_modulename.

   Explore more on: https://thestempedia.com/docs/dabble/game-pad-module/
*/
#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <DabbleESP32.h>

// Setup joystick
void setup_joystick() {
    Dabble.begin("Haloball");  // set bluetooth name of your device
}

void joystick_loop() {
    float scale;

    Dabble.processInput();    // this function is used to refresh data obtained from smartphone.Hence calling this function is mandatory in order to get data properly from your mobile.

    int xAxisValue = GamePad.getXaxisData();
    int yAxisValue = GamePad.getYaxisData();
    // Serial.printf("X: %d | Y: %d\n", xAxisValue, yAxisValue);   // debug purpose

    if (GamePad.isTrianglePressed()) {
        scale = 1.5;
    } else {
        scale = 1;
    }
    
    // Forward
    if (xAxisValue >= -1 && xAxisValue <= 1 && yAxisValue >= 3) {
        linear_vel = 0.2 * scale;
        angular_vel = 0;
        last_cmd_vel_time = millis();
    }
    // Turn right while moving forward
    else if (xAxisValue >= 3 && yAxisValue >= 3) {
        linear_vel = 0.15 * scale;
        angular_vel = -2 * scale;
        last_cmd_vel_time = millis();
    }
    // Turn left while moving forward
    else if (xAxisValue <= -3 && yAxisValue >= 3) {
        linear_vel = 0.15 * scale;
        angular_vel = 2 * scale;
        last_cmd_vel_time = millis();
    }
    // Reverse
    else if (xAxisValue >= -1 && xAxisValue <= 1 && yAxisValue <= -3) {
        linear_vel = -0.2 * scale;
        angular_vel = 0;
        last_cmd_vel_time = millis();
    }
    // Turn right while reversing
    else if (xAxisValue >= 3 && yAxisValue <= -3) {
        linear_vel = -0.15 * scale;
        angular_vel = 2 * scale;
        last_cmd_vel_time = millis();
    }
    // Turn left while reversing
    else if (xAxisValue <= -3 && yAxisValue <= -3) {
        linear_vel = -0.15 * scale;
        angular_vel =-2 * scale;
        last_cmd_vel_time = millis();
    }
    // Turn left
    else if (xAxisValue <= -3 && yAxisValue >= -1 && yAxisValue <= 1) {
        linear_vel = 0;
        angular_vel = 2 * scale;
        last_cmd_vel_time = millis();
    }
    // Turn right
    else if (xAxisValue >= 3 && yAxisValue >= -1 && yAxisValue <= 1) {
        linear_vel = 0;
        angular_vel = -2 * scale;
        last_cmd_vel_time = millis();
    }
}