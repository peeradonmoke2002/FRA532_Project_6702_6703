#include <Arduino.h>

// — encoder pin definitions —
// Connect your AMT103 A & B outputs here
const int encoderPinA = 32;  // GPIO32 → Channel A
const int encoderPinB = 33;  // GPIO33 → Channel B

// — state variables —
// encoderCount holds the current position (can be + or –)
volatile long encoderCount = 0;
// lastEncoded stores the previous (A,B) state as two bits
volatile byte lastEncoded = 0;

// — interrupt routine to update encoderCount —
// must be in IRAM on ESP32 for fastest response
void IRAM_ATTR updateEncoder() {
  // read both channelss
  byte MSB = digitalRead(encoderPinA);
  byte LSB = digitalRead(encoderPinB);
  byte encoded = (MSB << 1) | LSB;

  // combine previous and current state into a 4-bit number
  byte sum = (lastEncoded << 2) | encoded;

  // depending on the transition, increment or decrement
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    encoderCount++;
  }
  else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    encoderCount--;
  }

  // save this state for next interrupt
  lastEncoded = encoded;
}

void setup() {
  // start serial for output
  Serial.begin(115200);

  // configure pins with pull-ups (encoder outputs are open-collector)
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);

  // read the initial state of encoder pins
  byte MSB = digitalRead(encoderPinA);
  byte LSB = digitalRead(encoderPinB);
  lastEncoded = (MSB << 1) | LSB;

  // attach interrupts on both channels
  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), updateEncoder, CHANGE);
}

void loop() {
  // capture and print the count every 100ms
  static long lastPrint = 0;
  if (millis() - lastPrint > 100) {
    lastPrint = millis();
    noInterrupts();
    long pos = encoderCount;
    interrupts();

    Serial.print("Encoder count = ");
    Serial.println(pos);
  }