#include <Arduino.h>
int CWcount = 0;
String message;

int AState, BState;
int lastAState = LOW;
int lastBState = LOW;

unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

void setup() {
    // Start serial
    Serial.begin(9600);

    //configure pin 7 as an input and enable the internal pull-up resistor
    pinMode(7, INPUT_PULLUP);

    //configure pin 4 as an input and enable the internal pull-up resistor
    pinMode(4, INPUT_PULLUP);    // set pin to input

    message = "Start " + String(CWcount);
    Serial.println(message);
    //delay(1);        // delay in between reads for stability
}

void loop() {
    // put your main code here, to run repeatedly:
    int directionValA = digitalRead(7);
    int directionValB = digitalRead(4);

    if (directionValA != lastAState || directionValB != lastBState) {
        // reset the debouncing timer
        lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > debounceDelay) {
        // whatever the reading is at, it's been there for longer than the debounce
        // delay, so take it as the actual current state:

        // Check if the state has changed:
        if (directionValA != AState) {
            AState = directionValA;
        }
        if (directionValB != BState) {
            BState = directionValB;
        }

    }

    if(directionValA == HIGH && directionValB == LOW) {
        CWcount--;
        message = "CCW";
        Serial.println(message);
    }
    else if(directionValA == LOW && directionValB == HIGH) {
        CWcount++;
        message = "CW";
        Serial.println(message);
    }

    // Save current reading
    lastAState = directionValA;
    lastBState = directionValB;


    //delay(1);        // delay in between reads for stability

}