#include <Arduino.h>

// Calculate error (error = reference - desired)
// Ts = sample time
// Tc = current time
// cOutput = controller output

const float Kp = 0, Ki = 0, Kd = 0;
float I = 0, D = 0;
float error, pastError = 0;
unsigned long Ts = 0, Tc;
float cOutput, maxOutput = 255; // or 400 if using the motorShield header

void setup() {
  // Current time
  Tc = millis();
}

void loop() {
  // Read desired(reference) position
  // Read current(actual) position

  if(Ts > 0) {
    // Calculate D
    D = (error - pastError)/float(Ts);
    pastError = error;
  } else {
    D = 0;
  }
  // Calculate I
  I += float(Ts) * error;
  // Calculate controller output
  cOutput = Kp*error + Ki*I + Kd*D;

  // If calculated controller output is higher than the maximum output,
  // set output to the maximum output with matching sign
  if(abs(cOutput) > maxOutput) {
    if(cOutput < 0) cOutput = -1*maxOutput;
    else cOutput = maxOutput;
  }

  // TODO update the voltage output and send it to the motor


  // Set Ts
  Ts = millis()-Tc;
  Tc = millis();


}