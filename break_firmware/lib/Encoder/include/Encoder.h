// Encoder.h
#pragma once
#include <Arduino.h>

class Encoder {
public:
  Encoder(uint8_t pinA, uint8_t pinB);
  ~Encoder();

  void begin();
  long getCount();
  void reset();

private:
  uint8_t        _pinA, _pinB;
  volatile long  _count;
  volatile uint8_t _lastEncoded;

  static void IRAM_ATTR _handleISR(void* arg);
  void IRAM_ATTR update();
};
