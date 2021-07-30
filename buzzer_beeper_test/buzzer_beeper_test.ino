
#include "Arduino.h" 

#define D9 PB1
const int BZR = 9; // buzzer or beeper
const int BZR_FREQ = 261; // hz



void buzz() {
  // TODO: rewrite to be interrupt safe using millis()
  //tone(BZR, BZR_FREQ);
  digitalWrite(BZR,HIGH);
  delay(1000);
  digitalWrite(BZR,LOW);
  //noTone(BZR);
  Serial.println("buzz");
}




void setup() {
  // put your setup code here, to run once:
  pinMode(BZR, OUTPUT);
  Serial.begin(115200);
  Serial.println("Timer online");

}

void loop() {
  // put your main code here, to run repeatedly:
    buzz();
    delay(500);
}
