#define CH1 27 
#define CH2 26
#define CH3 25
#define CH4 33
#define CH5 32
#define CH6 35
#define LED 2

void setup() {
  Serial.begin(115200);
  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);
  pinMode(CH3, INPUT);
  pinMode(CH4, INPUT);
  pinMode(CH5, INPUT);
  pinMode(CH6, INPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
}

void loop() {
  Serial.print(digitalRead(CH1));
  Serial.print(",");
  Serial.print(digitalRead(CH2));
  Serial.print(",");
  Serial.print(digitalRead(CH3));
  Serial.print(",");
  Serial.print(digitalRead(CH4));
  Serial.print(",");
  Serial.print(digitalRead(CH5));
  Serial.print(",");
  Serial.print(digitalRead(CH6));
  Serial.println();
  delay(1);
}
