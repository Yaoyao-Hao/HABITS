
const byte switchPin       = 4; // ToggerSwitch pin to start/pause experiment
const byte ledPin          = 13; // LED pin
bool ledState = LOW;

void setup() {
  Serial.begin(115200);   // initialize seiral for debugging
  pinMode(ledPin, OUTPUT);
  pinMode(switchPin, INPUT_PULLUP); // low if switch on; hight if switch off

}

void loop() {
  if (digitalRead(switchPin) == 0) { 
    digitalWrite(ledPin, HIGH);
  } else {
    // Flashing LED
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }
    digitalWrite(ledPin, ledState);
    delay(500);
  }

}
