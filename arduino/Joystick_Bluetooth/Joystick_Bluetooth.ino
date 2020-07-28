#include <SoftwareSerial.h>
#define BT_RX 8
#define BT_TX 9

SoftwareSerial bluetooth(BT_RX, BT_TX);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(1000000);
  bluetooth.begin(1000000);
}

void loop() {
  // put your main code here, to run repeatedly:
 int X = analogRead(0);
 int Y = analogRead(1);
 X = map(X, 0, 1023, -30, 30);
 X = constrain(X, -30, 30);
 Y = map(Y, 0, 1023, -30, 30);
 Y = constrain(Y, -30, 30);
 
 Serial.print("X value is :");
 Serial.println(X);
 
 
 Serial.print("Y value is :");
 Serial.println(Y); 

  if(bluetooth.available())
    Serial.write(bluetooth.read());
  if(Serial.available())
    bluetooth.write(Serial.read());
}
