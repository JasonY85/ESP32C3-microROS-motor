#include <Arduino.h>
#include "config.h"
#include "encoder.h"

// Define local variables with volatile for interrupt safety
volatile int left_encoder_count = 0;
volatile int right_encoder_count = 0;

// Encoder state tracking variables
volatile uint8_t left_encoder_state = 0;
volatile uint8_t right_encoder_state = 0;

// Interrupt handlers for encoders with quadrature decoding
void IRAM_ATTR left_encoder_isr() {
  static uint8_t old_state = 0;
  uint8_t new_state = (digitalRead(ENCODER0_PINA) << 1) | digitalRead(ENCODER0_PINB);
  
  // State transition table for quadrature decoding
  if ((old_state == 0x00 && new_state == 0x01) ||
      (old_state == 0x01 && new_state == 0x03) ||
      (old_state == 0x03 && new_state == 0x02) ||
      (old_state == 0x02 && new_state == 0x00)) {
    left_encoder_count++;
  } else if ((old_state == 0x00 && new_state == 0x02) ||
             (old_state == 0x02 && new_state == 0x03) ||
             (old_state == 0x03 && new_state == 0x01) ||
             (old_state == 0x01 && new_state == 0x00)) {
    left_encoder_count--;
  }
  
  old_state = new_state;
}

void IRAM_ATTR right_encoder_isr() {
  static uint8_t old_state = 0;
  uint8_t new_state = (digitalRead(ENCODER1_PINA) << 1) | digitalRead(ENCODER1_PINB);
  
  // State transition table for quadrature decoding
  if ((old_state == 0x00 && new_state == 0x01) ||
      (old_state == 0x01 && new_state == 0x03) ||
      (old_state == 0x03 && new_state == 0x02) ||
      (old_state == 0x02 && new_state == 0x00)) {
    right_encoder_count++;
  } else if ((old_state == 0x00 && new_state == 0x02) ||
             (old_state == 0x02 && new_state == 0x03) ||
             (old_state == 0x03 && new_state == 0x01) ||
             (old_state == 0x01 && new_state == 0x00)) {
    right_encoder_count--;
  }
  
  old_state = new_state;
}

// Setup encoders with full quadrature support
void setup_encoders() {
  pinMode(ENCODER0_PINA, INPUT_PULLUP);
  pinMode(ENCODER0_PINB, INPUT_PULLUP);
  pinMode(ENCODER1_PINA, INPUT_PULLUP);
  pinMode(ENCODER1_PINB, INPUT_PULLUP);

  // Attach interrupts to both edges of both channels
  attachInterrupt(digitalPinToInterrupt(ENCODER0_PINA), left_encoder_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER0_PINB), left_encoder_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER1_PINA), right_encoder_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER1_PINB), right_encoder_isr, CHANGE);
}

// Calculate speed in rad/s from encoder counts
float calculate_speed(int encoder_count, float delta_time) {
  float revolutions = encoder_count / (float)ENCODER_CPR;
  float rad_per_sec = (revolutions * 2 * PI) / delta_time;
  return rad_per_sec;
}