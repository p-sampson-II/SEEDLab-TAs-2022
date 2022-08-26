#include "Arduino.h"

// variables for computation
int counter = 0;
bool docounting = false;

// varaibles for serial communication
String InputString = ""; // a string to hold incoming data
bool StringComplete = false;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  // reserve 200 bytes for the inputString:
  InputString.reserve(200);
  Serial.println("Ready!"); // Let anyone on the other end of the serial line know that Arduino is ready

}


void loop() {
  // Change behavior based on serial input
  if (StringComplete) {
    switch (InputString.charAt(0)) {
      case 'S':
        docounting = true;
        break;
      case 'E':
        docounting = false;
        break;
    }
    StringComplete = false;
  }

  // Some main code
  if (docounting) {
    counter++;
    if (counter >= 100) {
      Serial.println("Finished");
      docounting = false;
    }
  }
  else {
    counter = 0;
  }

  Serial.print(millis());
  Serial.print("\t");
  Serial.print(counter);
  Serial.println("");

  // wait 100 ms
  delay(100);

}


/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    InputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      StringComplete = true;
    }
  }
}
