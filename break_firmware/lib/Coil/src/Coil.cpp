#include "Coil.h"

Coil::Coil(int pin1, int pin2, int ch1, int ch2, int freq, int resolution)
  : _pin1(pin1), _pin2(pin2),
    _ch1(ch1),   _ch2(ch2),
    _freq(freq), _res(resolution)
{}

void Coil::begin() {
  // configure channel 1
  ledcSetup(_ch1, _freq, _res);
  ledcAttachPin(_pin1, _ch1);

  // configure channel 2
  ledcSetup(_ch2, _freq, _res);
  ledcAttachPin(_pin2, _ch2);

  // ensure coils start off
  off();
}

void Coil::on(uint32_t duty) {
  ledcWrite(_ch1, duty);
  ledcWrite(_ch2, duty);
}

void Coil::off() {
  ledcWrite(_ch1, 0);
  ledcWrite(_ch2, 0);
}
