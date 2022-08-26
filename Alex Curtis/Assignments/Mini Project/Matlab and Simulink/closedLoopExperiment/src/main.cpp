// Alex Curtis
#include <Arduino.h>
#include <DualMC33926MotorShield.h>
#include <Encoder.h>
#define ENCODER_OPTIMIZE_INTERRUPTS         // Highly Optimized interrupt in background
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

// Controller parameters
int control(float current, float desired);
const float KP = 0.6137, KI = 0.0888, KD = 0;
float I = 0, D = 0, error, pastError = 0;
unsigned long Ts = 0, Tc = 0;
int targetSpeed = 0;
int desiredAngleCoeff = 2;            // Angle to move the motor to (Value between 0-3)
volatile float currentAngle = 0, desiredAngle = 0;    // position relative to initial position (radians)

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
bool runStepExperiment = false;
bool runControlExperiment = false;

String InputString = ""; // a string to hold incoming data


void setup() {
	// 115200
	Serial.begin(115200);
	InputString.reserve(200);      // reserve 200 bytes for the inputString
	motor.init();                       // Initialize motor
	motor.setM1Speed(0);         // Set motor speed to 0
	Serial.println("Ready!");           // Tell Matlab that Arduino is ready
}

void loop() {
	// Change behavior based on serial input
	if (commandReceived) {
		switch (InputString.charAt(0)) {
			case 'S':
				runControlExperiment = false;
				runStepExperiment = true;
				start = millis();
				break;
			case 'C':
				runStepExperiment = false;
				runControlExperiment = true;
				Tc = millis();
				break;
			case 'E':
				runStepExperiment = false;
				runControlExperiment = false;
				break;
		}
		commandReceived = false;
	}

	if(runStepExperiment && !runControlExperiment) {
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
			motor.setM1Speed(0);
			motorEnc.write(0);  // Reset encoders
			runStepExperiment = false;
		}
	}
	else if(!runStepExperiment && runControlExperiment) {
		// todo set up control experiment

		// Read encoders and update current time
		newPosition = motorEnc.read();
		//while (newPosition >= 3200) newPosition -= 3200; // MODIFIED SO 3200 COUNTS CHANGES TO 0!!!!!!

		// Find current angular motor position relative to initial position
		currentAngle = float(newPosition - initialPosition) * float((2.0 * PI) / float(CPR));

		// Calculate desired angular position
		desiredAngle = desiredAngleCoeff * float(PI / 2);

		// Find current angular velocity in rad/s: (x2 - x1) / ∆t
		angVelocity = ((float((newPosition - initialPosition) - (position - initialPosition)) * float((2.0 * PI) / CPR)) *float(1000)) / float(Ts);

		// use control() to determine target speed and direction
		targetSpeed = control(currentAngle, desiredAngle);

		// Set the motor to the target speed (also accounts for direction)
		motor.setM1Speed(targetSpeed);

		// Print elapsed time, target speed, angular velocity
		Serial.print(millis()-start); // Prints elapsed time in ms
		Serial.print(" ");
		Serial.print(currentAngle);
		Serial.print(" ");
		Serial.print(desiredAngle);
		Serial.println(" ");
		Serial.println(error);
		Serial.print(" ");
		Serial.print((TARGET_SPEED) * 51 / 80); // Same conversion used in the motor library for analogWrite()
		Serial.print(" ");
		Serial.print(angVelocity,7);
		Serial.print(" ");
		Serial.print(angPosition,7);
		Serial.println("");

		// if the wheel gets to the desired position, stop
		if(error < 5 || millis()-start > 100000) {
			Serial.println("Finished");
			runControlExperiment = false;
		}
		// Print elapsed time, target speed, angular velocity
		//Serial.print(millis() - start); // Prints elapsed time in ms
		//Serial.print("\t");
	}

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

// Current and desired are angles in radians
int control(float current, float desired) {
	double newTargetSpeed, maxSpeed = 400, minSpeed = -400;   // 400 if using the motorShield header
	error = desired - current;      // Update error

	// Calculate D
	if (Ts > 0) {
		D = (error - pastError) / float(Ts);
		pastError = error;
	} else {
		D = 0;
	}

	// TODO when should I be reset to 0?????
	// If I keeps accumulating, so will the controller output, and the speed will increase to max and stay there.
	// Should it be reset when error gets close to zero?
	// Or should it only be reset when the desired position changes?
	// Or should I just remove I entirely from the controller?

	// Calculate I
	I += float(Ts) * error;

	// Calculate controller output
	newTargetSpeed = KP * error + KI * I + KD * D;

	// TODO make sure this is correct
	// The step response experiment used 255 as the max command.
	// So the controller output needs to be scaled by 400/255
	newTargetSpeed *= 400.0 / 255.0;

	// If calculated controller output is higher than the maximum output,
	// set output to the maximum output with appropriate sign
	if (abs(newTargetSpeed) > maxSpeed) {
		if (newTargetSpeed < 0) newTargetSpeed = minSpeed;
		else newTargetSpeed = maxSpeed;
		I -= float(Ts) * error; // Undo integration
	}

//	Serial.print("\nCurrentAngle: ");
//	Serial.print(current);
//	Serial.print("\tDesiredAngle: ");
//	Serial.print(desired);
//	Serial.println("\terror: ");
//	Serial.println(error);
//	Serial.print("\tcontroller: ");
//	Serial.println(newTargetSpeed);
//	Serial.print("\tcontroller(int): ");
//	Serial.println(int(newTargetSpeed));

	Ts = millis() - Tc; // Determine sample time
	Tc = millis();      // Update current time

	// newTargetSpeed = newTargetSpeed*50; // TODO should this be scaled????

	return int(newTargetSpeed);
}