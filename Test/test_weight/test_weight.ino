/*
 Example using the SparkFun HX711 breakout board with a scale
 By: Nathan Seidle
 SparkFun Electronics
 Date: November 19th, 2014
 License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).
 
 This is the calibration sketch. Use it to determine the calibration_factor that the main example uses. It also
 outputs the zero_factor useful for projects that have a permanent mass on the scale in between power cycles.
 
 Setup your scale and start the sketch WITHOUT a weight on the scale
 Once readings are displayed place the weight on the scale
 Press +/- or a/z to adjust the calibration_factor until the output readings match the known weight
 Use this calibration_factor on the example sketch
 
 This example assumes pounds (lbs). If you prefer kilograms, change the Serial.print(" lbs"); line to kg. The
 calibration factor will be significantly different but it will be linearly related to lbs (1 lbs = 0.453592 kg).
 
 Your calibration factor may be very positive or very negative. It all depends on the setup of your scale system
 and the direction the sensors deflect from zero state

 This example code uses bogde's excellent library: https://github.com/bogde/HX711
 bogde's library is released under a GNU GENERAL PUBLIC LICENSE

 Arduino pin 2 -> HX711 CLK
 3 -> DOUT
 5V -> VCC
 GND -> GND
 
 Most any pin on the Arduino Uno will be compatible with DOUT/CLK.
 
 The HX711 board can be powered from 2.7V to 5V so the Arduino 5V power should be fine.
 
*/

#include "HX711.h"

#define DOUT  7
#define CLK  8

HX711 scale;
unsigned long startMillis;
unsigned long currentMillis;
float calibration_factor = 6470; //6470

void setup() {
  scale.begin(DOUT, CLK);
  delay(3000);
  Serial.begin(115200);
  Serial.println();
  Serial.println("Calibration Starts: remove all weight from scale!");
  delay(5000);
  Serial.println();
  Serial.println("After readings starts, place known weight on scale");
  Serial.println();
  Serial.println("Press + or a to increase calibration factor");
  Serial.println("Press - or z to decrease calibration factor");
  Serial.println();

  scale.set_scale();
  scale.tare();	//Reset the scale to 0

  long zero_factor = scale.read_average(); //Get a baseline reading
  Serial.print("Zero factor: "); //This can be used to remove the need to tare the scale. Useful in permanent scale projects.
  Serial.println(zero_factor);
   scale.tare();  //Reset the scale to 0
}

void loop() {

  scale.set_scale(calibration_factor); //Adjust to this calibration factor

  Serial.print("Reading: ");
  //startMillis = micros();
  Serial.print(scale.get_units(), 2);//10Hz:0.07 ms; 20Hz: 41 ms; 50Hz: 70 ms; 100Hz: 80 ms; 1000Hz: 90 ms WAIT UNTIL READY!!!
  //Serial.println(micros() - startMillis); // max: 6 ms; >100Hz-read: 5-6ms; 65Hz-read: <1.5 ms; <=50Hz-read: 0.2 ms
  Serial.print(" g,  calibration_factor: "); //Change this to kg and re-adjust the calibration factor if you follow SI units like a sane person
  Serial.println(calibration_factor);

  if(Serial.available())
  {
    char temp = Serial.read();
    if(temp == '+' || temp == 'a')
      calibration_factor += 10;
    else if(temp == '-' || temp == 'z')
      calibration_factor -= 10;
  }
  delay(1000);
}
