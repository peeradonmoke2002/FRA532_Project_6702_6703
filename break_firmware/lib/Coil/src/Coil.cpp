#include "Coil.h"

/**
 * @brief Initialize both ESP32 LEDC channels for driving the coils.
 *
 * Uses:
 *  - _pin1/_pin2: GPIO pins to output PWM
 *  - _ch1/_ch2:   LEDC channels (0–15)
 *  - _freq:       PWM frequency
 *  - _res:        PWM resolution
 */
void Coil::begin() {
  // configure channel 1
  ledcSetup(_ch1, _freq, _res);
  ledcAttachPin(_pin1, _ch1);

  // configure channel 2
  ledcSetup(_ch2, _freq, _res);
  ledcAttachPin(_pin2, _ch2);

  // ensure we start with coils off
  off();
}

/**
 * @brief Turn both coils on at the given duty cycle.
 * @param duty Range = 0 … (2^_res – 1)
 *             0 → fully off, max → fully on
 */
void Coil::on(uint32_t duty) {
  ledcWrite(_ch1, duty);  // apply duty to channel 1/pin1
  ledcWrite(_ch2, duty);  // apply duty to channel 2/pin2
}

/**
 * @brief Turn both coils fully off (duty = 0).
 */
void Coil::off() {
  ledcWrite(_ch1, 0);     // channel 1 = 0%
  ledcWrite(_ch2, 0);     // channel 2 = 0%
}
