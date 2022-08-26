int CWcount = 0;
String message;

int clkState = HIGH, dtState = HIGH;
int lastClkState = HIGH;
int lastDtState = HIGH;

unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

void setup() {
    // Start serial
    Serial.begin(9600);

    //configure pin 7 as an input and enable the internal pull-up resistor
    pinMode(7, INPUT_PULLUP);

    //configure pin 4 as an input and enable the internal pull-up resistor
    pinMode(4, INPUT_PULLUP);    // set pin to input

    message = "Start ";
    Serial.print(message);
    Serial.println(CWcount);
    //delay(1);        // delay in between reads for stability
}

void loop() {
    // put your main code here, to run repeatedly:
    int clk = digitalRead(4);
    int dt = digitalRead(7);

    if (clk != lastClkState || dt != lastDtState) {
        // reset the debouncing timer
        lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > debounceDelay) {
        // whatever the reading is at, it's been there for longer than the debounce
        // delay, so take it as the actual current state:

        // Check if the state has changed:
        if (clk != clkState) {
            clkState = clk;
        }
        if (dt != dtState) {
            dtState = dt;
        }

        if(clkState == LOW) {
            CWcount--;
            message = "CCW";
        }
        else if(dtState == LOW) {
            CWcount++;
            message = "CW";
        }
        if(clkState != lastClkState || dtState != lastDtState) {
            Serial.print(message + " ");
            Serial.println(CWcount);
        }
        

    }
    // Save current reading
    lastClkState = clk;
    lastDtState = dt;
    //delay(1);        // delay in between reads for stability

}
