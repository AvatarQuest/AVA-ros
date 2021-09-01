#include <Arduino.h>
#include <Servo.h>
#include <RotaryEncoder.h>

#define PIN_IN1 7
#define PIN_IN2 8
#define PITCH 11

Servo pitch_motor;

// Setup a RotaryEncoder with 4 steps per latch for the 2 signal input pins:
// RotaryEncoder encoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::FOUR3);

// Setup a RotaryEncoder with 2 steps per latch for the 2 signal input pins:
RotaryEncoder encoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::TWO03);


//#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO_EVERY)
// This interrupt routine will be called on any change of one of the input signals
void checkPosition()
{
  Serial.print("Updating postition");
  encoder.tick(); // just call tick() to check the state.
}



void setup()
{
  pitch_motor.attach(PITCH);
  Serial.begin(9600);
  while (!Serial);
  Serial.println("InterruptRotator example for the RotaryEncoder library.");

//  attachInterrupt(digitalPinToInterrupt(PIN_IN1), checkPosition, CHANGE);
//  attachInterrupt(digitalPinToInterrupt(PIN_IN2), checkPosition, CHANGE);
} // setup()


// Read the current position of the encoder and print out when changed.
void loop()
{
  static int pos = 0;
  encoder.tick();
//  pitch_motor.write(86);

  int newPos = encoder.getPosition();
  if (pos != newPos) {
    Serial.print("\npos:");
    Serial.print(newPos);
//    Serial.wri(" dir:");
//    Serial.println((int)(encoder.getDirection()));
    pos = newPos;
  } // if
} // loop ()
