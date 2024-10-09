/*Copyright (c) 2010 bildr community

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
*/

# include "MPR121.h"


//int irqpin = 2;  // Digital 2
boolean touchStates[12]; //to keep track of the previous touch states

MPR121 cap = MPR121(0x5A);
uint8_t currtouched_8 = 0;
uint16_t currtouched = 0;
unsigned int time_start;

void setup() {
  //pinMode(irqpin, INPUT);
  //digitalWrite(irqpin, HIGH); //enable pullup resistor
  delay(3000);
  Serial.begin(9600);
//  while (!Serial) { // needed to keep leonardo/micro from starting too fast!
//    delay(10);
//    Serial.print("11");
//  }

  Wire.begin();
  Wire.setClock(1000000);

  Serial.println("Adafruit MPR121 Capacitive Touch sensor test");

  // Default address is 0x5A, if tied to 3.3V its 0x5B
  // If tied to SDA its 0x5C and if SCL then 0x5D
  //  while (!cap.begin()) {
  //    Serial.println("MPR121 not found, check wiring?");
  //    delay(1000);
  //  }
  cap.begin(5);
  Serial.println("MPR121 found!");



  //mpr121_setup();
}

void loop() {
  readTouchInputs();
  delay(1000);
  Serial.println("reading...");
}


void readTouchInputs() {

  //    //read the touch state from the MPR121
  //    Wire.requestFrom(0x5A,2);
  //
  //    byte LSB = Wire.read();
  //    byte MSB = Wire.read();
  //
  //    Serial.println(LSB);
  //    Serial.println(MSB);
  //
  //    uint16_t touched_1 = ((MSB << 8) | LSB); //16bits that make up the touch states

  time_start = micros();
  //currtouched = cap.touched_state_12();
  currtouched_8 = cap.touched_state_8();// 30 us
  //Serial.println(currtouched_8);
  Serial.println(micros() - time_start);


  for (int i = 0; i < 8; i++) { // Check what electrodes were pressed
    if (currtouched_8 & (1 << i)) {

      if (touchStates[i] == 0) {
        //pin i was just touched
        Serial.print("pin ");
        Serial.print(i);
        Serial.println(" was just touched");

      } else if (touchStates[i] == 1) {
        //pin i is still being touched
      }

      touchStates[i] = 1;
    } else {
      if (touchStates[i] == 1) {
        Serial.print("pin ");
        Serial.print(i);
        Serial.println(" is no longer being touched");

        //pin i is no longer being touched
      }

      touchStates[i] = 0;
    }

  }
}
