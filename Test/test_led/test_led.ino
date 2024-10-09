void setup() {
  // put your setup code here, to run once:
  // left
  pinMode(2, OUTPUT); //Sound tPWM1
  pinMode(5, OUTPUT); //led pwm1
  
  // top
  pinMode(3, OUTPUT); //Sound tPWM2
  pinMode(6, OUTPUT); //R pwm2
  pinMode(9, OUTPUT); //G pwm3
  pinMode(10, OUTPUT); //B pwm4

  // right
  pinMode(16, OUTPUT); //Sound tPWM3
  pinMode(20, OUTPUT); //led pwm5
}

void loop() {
  // put your main code here, to run repeatedly:
  // left
  analogWriteFrequency(gpSMART_tPWM_Lines[tPWM_num - 1], tPWM_frequency[tPWM_num - 1][freq_num - 1]);
    analogWrite(gpSMART_tPWM_Lines[tPWM_num - 1], tPWM_duty[tPWM_num - 1][freq_num - 1]);
  analogWrite(2,10);
  digitalWrite(5,0);
  delay(500);
  analogWrite(2,0);
  digitalWrite(5,1);
  delay(500);
  analogWrite(2,0);
  digitalWrite(5,0);
  delay(500);

  // middle
  analogWrite(3,10);
  digitalWrite(6,0);
  digitalWrite(9,0);
  digitalWrite(10,0);
  delay(500);
    analogWrite(3,0);
  digitalWrite(6,1);
  digitalWrite(9,0);
  digitalWrite(10,0);
  delay(500);
    analogWrite(3,0);
  digitalWrite(6,0);
  digitalWrite(9,1);
  digitalWrite(10,0);
  delay(500);
    analogWrite(3,0);
  digitalWrite(6,0);
  digitalWrite(9,0);
  digitalWrite(10,1);
  delay(500);
  digitalWrite(10,0);
  delay(500);

  // right
  analogWrite(16,10);
  digitalWrite(20,0);
  delay(500);
  analogWrite(16,0);
  digitalWrite(20,1);
  delay(500);
  analogWrite(16,0);
  digitalWrite(20,0);
  delay(500);
}
