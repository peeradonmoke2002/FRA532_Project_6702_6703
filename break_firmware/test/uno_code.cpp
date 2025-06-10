#include <Arduino.h>

const int COIL1_PIN = 25;
const int COIL2_PIN = 26;
const int PWM_CH1    = 0;
const int PWM_CH2    = 1;
const int PWM_FREQ   = 1000;  // 1 kHz
const int PWM_RES    = 8;     // 8-bit
const int DUTY_50    = 120;   // 50% duty max 225

// Pulse parameters (in ms):
const uint16_t ON_TIME_MS  = 10;  // coil on for 10 ms
const uint16_t OFF_TIME_MS = 5;  // coil off for 20 ms

void setup() {
  ledcSetup(PWM_CH1, PWM_FREQ, PWM_RES);
  ledcAttachPin(COIL1_PIN, PWM_CH1);

  ledcSetup(PWM_CH2, PWM_FREQ, PWM_RES);
  ledcAttachPin(COIL2_PIN, PWM_CH2);
}


void loop() {
  // — ON phase —
  ledcWrite(PWM_CH1, DUTY_50);
  ledcWrite(PWM_CH2, DUTY_50);
  delay(ON_TIME_MS);

  // — OFF phase —
  ledcWrite(PWM_CH1, 0);
  ledcWrite(PWM_CH2, 0);
  delay(OFF_TIME_MS);
}