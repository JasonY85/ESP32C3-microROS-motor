#include <Arduino.h>
#include "config.h"
#include "motor_control.h"

void setup_motors() {
  // Initialize motor control pins
  pinMode(STBY, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  //Wake motor from standby
  digitalWrite(STBY, HIGH);

  // Stop motors initially
  set_motor_speed(PWMA, AIN1, AIN2, 0);
  set_motor_speed(PWMB, BIN1, BIN2, 0);
}

// Set motor speed (PWM + direction)
void set_motor_speed(int pwm_pin, int in1_pin, int in2_pin, int speed) {
  speed = constrain(speed, MIN_PWM, MAX_PWM);
  if (speed > 0) {
    digitalWrite(in1_pin, HIGH);
    digitalWrite(in2_pin, LOW);
  } else if (speed < 0) {
    digitalWrite(in1_pin, LOW);
    digitalWrite(in2_pin, HIGH);
  } else {
    digitalWrite(in1_pin, LOW);
    digitalWrite(in2_pin, LOW);
  }
  analogWrite(pwm_pin, abs(speed));
}
