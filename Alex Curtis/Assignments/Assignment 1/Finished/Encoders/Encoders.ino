// Alex Curtis
// Encoders without ISR
// Finished

// To practice using a 5-pin rotary encoder.
// Pin 3 is connected to CLK
// Pin 6 is connected to DT
// GND is connected to GND
// 5v is connected to the +
// A 6 pF capacitor is placed between the GND and CLK pins

// Pins used
#define CLK_PIN 3
#define DT_PIN 6

int position = 0;   // Stores a count that represents net rotation
String spin;        // Stores the direction to print
int lastClk = HIGH; // Stores the previous state of the clk input
int lastDt = HIGH;  // Stores the previous state of the dt input

void setup() {
    // Start serial
    Serial.begin(9600);

    // Set the clk and dt pins as inputs with the pullup resistors enabled
    pinMode(DT_PIN, INPUT_PULLUP);
    pinMode(CLK_PIN, INPUT_PULLUP);

    // Print an initial message
    Serial.print("Start ");
    Serial.println(position);
//    Serial.println("<Direction, lastCLK, lastDT, ");
}

void loop() {

    // Read the current values of clk and dt
    int clk = digitalRead(CLK_PIN);
    int dt = digitalRead(DT_PIN);

    // If dt changed to a logic 1
    if(dt != lastDt && dt == LOW) {

      // If dt is logic 0(inverted bc of pullup resistor), the encoder was rotated counterclockwise
      if (clk != dt) {
        position--;
        spin = "CCW";
      }
      // Otherwise, the encoder was rotated clockwise
      else {
        position++;
        spin = "CW";
      }

      // Print spin direction and count
      Serial.print("Turned: ");
      Serial.println(spin);

      // Print STATES for testing (Inverted for pullup)
      Serial.print("Last, Current: ");
      Serial.print(!lastClk);
      Serial.print(!lastDt);

      Serial.print("->");
      Serial.print(!clk);
      Serial.println(!dt);

      Serial.print("Position: ");
      Serial.println(position);

      Serial.println();
    }

    // Save current input to compare with the next iteration of the loop
    lastClk = clk;
    lastDt = dt;

    // Wait 1ms to help with bounce
    delay(1);
}
