int CWcount = 0;
String message;

int AState, Bstate, lastAState = LOW, lastBState= LOW;
//lastAState = lastBState = LOW;

unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

void setup() {
  // Start serial
  Serial.begin(9600);
  
  //configure pin 7 as an input and enable the internal pull-up resistor
  pinMode(7, INPUT_PULLUP);    
  
  //configure pin 4 as an input and enable the internal pull-up resistor
  pinMode(4, INPUT_PULLUP);    // set pin to input
  
  message = "Start " + CWcount;
  Serial.println(message);
  //delay(1);        // delay in between reads for stability
}

void loop() {
  // put your main code here, to run repeatedly:
  int directionValA = digitalRead(7);
  int directionValB = digitalRead(4);

  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }
  
  if(directionValA == HIGH && directionValB == LOW) {
    CWcount--;
    //message = "CCW " + CWcount;
    message = "CCW";
    Serial.println(message);
    //Serial.println(CWcount);
    //delay(1);        // delay in between reads for stability
  }
  else if(directionValA == LOW && directionValB == HIGH) {
    CWcount++;
    //message = "CW " + CWcount;
    
    message = "CW";
    Serial.println(message);
    //Serial.println(CWcount);
    
  }
  
  
  //delay(1);        // delay in between reads for stability
  
}
