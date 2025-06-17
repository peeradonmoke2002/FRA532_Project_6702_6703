// Encoder.cpp
#include "Encoder.h"

Encoder::Encoder(uint8_t pinA, uint8_t pinB)
 : _pinA(pinA), _pinB(pinB), _count(0), _lastEncoded(0)
{}

Encoder::~Encoder() {
  detachInterrupt(digitalPinToInterrupt(_pinA));
  detachInterrupt(digitalPinToInterrupt(_pinB));
}

void Encoder::begin() {
  pinMode(_pinA, INPUT_PULLUP);
  pinMode(_pinB, INPUT_PULLUP);
  _lastEncoded = (digitalRead(_pinA) << 1) | digitalRead(_pinB);

  attachInterruptArg(digitalPinToInterrupt(_pinA), _handleISR, this, CHANGE);
  attachInterruptArg(digitalPinToInterrupt(_pinB), _handleISR, this, CHANGE);
}

long Encoder::getCount() {
  noInterrupts();
    long val = _count;
  interrupts();
  return val;
}

void Encoder::reset() {
  noInterrupts();
    _count = 0;
  interrupts();
}

void IRAM_ATTR Encoder::_handleISR(void* arg) {
  static_cast<Encoder*>(arg)->update();
}

void IRAM_ATTR Encoder::update() {
  int MSB = digitalRead(_pinA);
  int LSB = digitalRead(_pinB);
  int encoded = (MSB << 1) | LSB;
  int sum     = (_lastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    _count++;
  }
  else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    _count--;
  }

  _lastEncoded = encoded;
}
