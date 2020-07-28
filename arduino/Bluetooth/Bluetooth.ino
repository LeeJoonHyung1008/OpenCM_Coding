#include <SoftwareSerial.h>
#define BT_RX 8
#define BT_TX 9

SoftwareSerial bluetooth(BT_RX, BT_TX);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  bluetooth.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(bluetooth.available())
    Serial.write(bluetooth.read());
  if(Serial.available())
    bluetooth.write(Serial.read());
}
