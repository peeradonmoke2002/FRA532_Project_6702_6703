#pragma once
#include <Arduino.h>

/**
 * @brief Simple PWM-driven dual-coil controller using ESP32 LEDC.
 */
class Coil {
public:
  /**
   * @param pin1       GPIO for coil 1 PWM output
   * @param pin2       GPIO for coil 2 PWM output
   * @param ch1        LEDC channel for coil 1 (0–15)
   * @param ch2        LEDC channel for coil 2 (0–15)
   * @param freq       PWM frequency in Hz
   * @param resolution PWM resolution in bits (e.g. 8 → duty range 0–255)
   */
  Coil(int pin1, int pin2, int ch1, int ch2, int freq, int resolution)
    : _pin1(pin1), _pin2(pin2),
      _ch1(ch1),   _ch2(ch2),
      _freq(freq), _res(resolution)
  {}

  /// Initialize LEDC channels and ensure coils are off
  void begin() {
    ledcSetup(_ch1, _freq, _res);
    ledcAttachPin(_pin1, _ch1);
    ledcSetup(_ch2, _freq, _res);
    ledcAttachPin(_pin2, _ch2);
    off();
  }

  /**
   * @brief Drive both coils at the given duty cycle.
   * @param duty PWM duty (0 → off, max = 2^_res–1 → full on)
   */
  void on(uint32_t duty) {
    ledcWrite(_ch1, duty);
    ledcWrite(_ch2, duty);
  }

  /// Turn both coils fully off (duty = 0)
  void off() {
    ledcWrite(_ch1, 0);
    ledcWrite(_ch2, 0);
  }

private:
  int _pin1, _pin2;   // GPIO pins
  int _ch1, _ch2;     // LEDC channels
  int _freq, _res;    // PWM parameters
};
