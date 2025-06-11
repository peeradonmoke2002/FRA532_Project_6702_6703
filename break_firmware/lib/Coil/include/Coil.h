#pragma once
#include <Arduino.h>

/**
 * @brief Simple PWM-driven dual-coil controller using ESP32 LEDC.
 */
class Coil {
public:
  /**
   * @param pin1       GPIO for coil #1 PWM output
   * @param pin2       GPIO for coil #2 PWM output
   * @param ch1        LEDC channel for coil #1 (0–15)
   * @param ch2        LEDC channel for coil #2 (0–15)
   * @param freq       PWM frequency in Hz
   * @param resolution PWM resolution in bits (e.g. 8 → duty range 0–255)
   */
  Coil(int pin1, int pin2, int ch1, int ch2, int freq, int resolution);

  /// Configure LEDC and ensure coils start off
  void begin();

  /**
   * @brief Drive both coils at the given duty cycle.
   * @param duty 0 → off, (2^resolution–1) → full on
   */
  void on(uint32_t duty);

  /// Turn both coils fully off
  void off();

private:
  int _pin1, _pin2;
  int _ch1, _ch2;
  int _freq, _res;
  
  // no custom destructor needed unless subclassing
};
