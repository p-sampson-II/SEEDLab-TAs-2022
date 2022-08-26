// Alex Curtis
#include <Arduino.h>
#include <DualMC33926MotorShield.h>
#include <Encoder.h>
// Program to perform a step response experiment on the motor
// Output a motor voltage command of 0, then  after 1 second
// Display the current time, motor voltage command, and angular velocity for
// each sampling interval.

// Default pins used in DualMotorShield
/*DualMC33926MotorShield::DualMC33926MotorShield()
{
  //Pin map
  _nD2 = 4;
  _M1DIR = 7;
  _M1PWM = 9;
  _M2DIR = 8;
  _M2PWM = 10;
  _nSF = 12;
  _M1FB = A0;
  _M2FB = A1;
}*/

// Encoder parameters
const float CPR = 50.0*64.0;           // Total encoder counts per revolution (CPR) of motor shaft
#define ENC_PIN_A 3       // Encoder output A (yellow wire, must be connected to 2 or 3 for interrupts)
#define ENC_PIN_B 2       // Encoder output B (white wire)
Encoder motorEnc(ENC_PIN_A,ENC_PIN_B); // 1st pin needs to be capable of interrupts (UNO: pin 2 or 3)
long initialPosition = 0;          // initial position of motor (counts)
volatile long position = 0;        // Previous position reading (counts)
volatile long newPosition = 0;     // Current position reading (counts)

// Experiment parameters
DualMC33926MotorShield motor;
const long SAMPLE_RATE = 5;     // Sample rate in ms
const int TARGET_SPEED = 400;   // Desired speed. If using DualMC33926MotorShield.h: max speed = 400
bool motorSet = false;
unsigned long now = 0;    // Elapsed time in ms
unsigned long start = 0;  // Experiment start time
volatile float angPosition = 0;     // position relative to initial position (radians)
volatile float angVelocity = 0;     // Current angular velocity (rad/s)

bool commandReceived = false; // flag for new serial input
bool runExperiment = false;

String InputString = ""; // a string to hold incoming data

void readNewData();

void setup() {
  // 115200
  Serial.begin(115200);
  InputString.reserve(200);      // reserve 200 bytes for the inputString
  motor.init();                       // Initialize motor
  motor.setM1Speed(0);         // Set motor speed to 0
  Serial.println("Ready!");           // Tell Matlab that Arduino is ready
  //runExperiment = true; // TODO remove
}

void loop() {
  // Change behavior based on serial input
  if (commandReceived) {
    switch (InputString.charAt(0)) {
    case 'S':
      runExperiment = true;
      start = millis();
      break;
    case 'E':
      runExperiment = false;
      break;
    }
    commandReceived = false;
  }

  if(runExperiment) {
    // At 1 second, set the motor to target speed
    if(millis()-start >= 1000 && !motorSet) {
      //initialPosition = motorEnc.read();  // Read initial position
      motor.setM1Speed(TARGET_SPEED);
      motorSet = true;
    }

    // Get new data every SAMPLE_TIME ms
    if (millis()-start >= now + SAMPLE_RATE) {

      // Adjust elapsed time
      now += SAMPLE_RATE;

      // Update readings
      // readNewData();
      // Read encoders
      newPosition = motorEnc.read();

      // Find current angular motor position relative to initial position
      angPosition = float( newPosition - initialPosition) * float( (2.0 * PI) / float(CPR));

      // Find current angular velocity in rad/s: (x2 - x1) / ∆t
      angVelocity = ( (float((newPosition - initialPosition) - (position - initialPosition))*float( (2.0 * PI)/CPR) )*float(1000) ) / float(SAMPLE_RATE);
      //angVelocity = ( (((newPosition - initialPosition) - (position - initialPosition))*long( (2000.0 * PI)/CPR) ) ) / SAMPLE_RATE;
      //angVelocity = (newPosition - initialPosition); //- (position - initialPosition));



      // If elapsed time is between 1s and 2s
      if(millis()-start >= 1000 && millis()-start <= 2000) {
        // Print elapsed time, target speed, angular velocity
        Serial.print(millis()-start); // Prints elapsed time in ms
        Serial.print("\t");
        Serial.print((TARGET_SPEED) * 51 / 80); // Same conversion used in the motor library for analogWrite()
        Serial.print("\t");
        Serial.print(angVelocity,7);
        Serial.print("\t");
        Serial.print(angPosition,7);
        Serial.println("");
      }
      // Update position with the new position
      position = newPosition;

    }

    // After 2s, turn off the motor and tell Matlab the experiment is done.
    if(millis()-start > 2000) {
      Serial.println("Finished");
      motor.setM1Speed(0);  //
      motorEnc.write(0);  // Reset encoders
      //TODO remove
      //runExperiment = false;
    }
  }

  // TODO should the reset be triggered by something else?
  // Reset position if there's an input from serial monitor
//  if(Serial.available()) {
//    Serial.read();
//    Serial.println("Position reset");
//    motorEnc.write(0);
//  }
}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    InputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      commandReceived = true;
    }
  }
}

void readNewData() {
  // Read encoders
  newPosition = motorEnc.read();

  // Find current angular motor position relative to initial position
  angPosition = ( newPosition - initialPosition) * long( (2.0 * PI) / long(CPR));

  // Find current angular velocity in rad/s: (x2 - x1) / ∆t
  angVelocity = ( (((newPosition - initialPosition) - (position - initialPosition))*long((2.0 * PI)/CPR) )*1000 ) / long(SAMPLE_RATE);

  // Update position with the new position
  position = newPosition;
}