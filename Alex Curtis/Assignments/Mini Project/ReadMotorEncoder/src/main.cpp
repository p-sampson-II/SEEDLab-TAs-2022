// Mini-Project: ReadMotorEncoder

// Program to read the rotary encoder on the motor
#include <Arduino.h>
#include <Encoder.h>  // https://www.pjrc.com/teensy/td_libs_Encoder.html

// Constants
//TODO: Verify position after 1 rotation is 64
const float CPR= 64*50;    // Total encoder counts per revolution of motor shaft
#define PIN_A 2   // Encoder output A (yellow wire, must be connected to 2 or 3 for interrupts)
#define PIN_B 3   // Encoder output B (white wire)

// Creates an Encoder object
Encoder motorEnc(PIN_A,PIN_B); // 1st pin needs to be capable of interrupts (UNO: pin 2 or 3)

long position = 0;     // position stores net rotation of motor (in counts)
long newPosition = 0;  // newPosition stores the updated position read
unsigned long now = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Start");
}

void loop() {
  // Read current motor position
  newPosition = motorEnc.read();

  // If the position changed, display the new position
  if (newPosition != position) {
    // Print position in counts
    Serial.print("Position(counts): ");
    Serial.print(newPosition);

    // Print angular position in radians
    Serial.print(" Position(rad): ");
    Serial.println(float(newPosition)*(2.0*PI)/CPR); // TODO: should this just be saved as a global variable?

    //Serial.print("Velocity: ");
    //Serial.println(( (((newPosition) - (position))*long( (2000.0 * PI)/CPR) ) ) / (millis()-now));

    // Save the updated position
    position = newPosition;
    now = millis();
  }

  // Reset position if there's an input from serial monitor
  if(Serial.available()) {
    Serial.read();
    Serial.println("Position reset");
    motorEnc.write(0);
  }

}