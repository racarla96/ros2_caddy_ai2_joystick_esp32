#define CH1 27 
#define CH2 26
#define CH3 25
#define CH4 33
#define CH5 32
#define CH6 35
#define LED 2

#include "PWMReader.hpp"

PWMReader ch1;
PWMReader ch2;
PWMReader ch3;
PWMReader ch4;
PWMReader ch5;
PWMReader ch6;

void error(){
  bool state = true;
  while(1){
    if(state) {digitalWrite(LED, HIGH); state = false;}
    else {digitalWrite(LED, LOW); state = true;}
    delay(200);
  }
} 

void setup() {
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  if (!ch1.begin(CH1)) error();
  if (!ch2.begin(CH2)) error();
  if (!ch3.begin(CH3)) error();
  if (!ch4.begin(CH4)) error();
  if (!ch5.begin(CH5)) error();
  if (!ch6.begin(CH6)) error();

  delay(2000);
  digitalWrite(LED, HIGH);
  delay(2000);
  digitalWrite(LED, LOW);
}

void loop() {
  Serial.print(ch3.isEnable());
  Serial.print(",");
  Serial.print(ch1.getDutyCicle_us());
  Serial.print(",");
  Serial.print(ch2.getDutyCicle_us());
  Serial.print(",");
  Serial.print(ch3.getDutyCicle_us());
  Serial.print(",");
  Serial.print(ch4.getDutyCicle_us());
  Serial.print(",");
  Serial.print(ch5.getDutyCicle_us());
  Serial.print(",");
  Serial.print(ch6.getDutyCicle_us());
  Serial.println();
  delay(20);
}
