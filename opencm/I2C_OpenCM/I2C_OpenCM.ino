/* Minimum_Source*/
#include <Wire.h>

byte x = 0;
byte y = 0;
int flag = 0;

void setup() {
  // put your setup code here, to run once:
  Wire.begin(14, 15);
}

void loop() {
  // put your main code here, to run repeatedly: 
  if(SerialUSB.read() == 's')
  {
    
  if(flag % 2 == 0)
  {
    x = 3;
    y = 255;
  }
  else
  {
    x = 0;
    y = 0;
  }
  Wire.beginTransmission(4); // transmit to device #4  slave id : 4
      Wire.write(x);              // sends one byte  
      Wire.write(y);              // sends one byte  
      Wire.endTransmission();    // stop transmitting

      SerialUSB.println(x * 256 + y);
      flag++;
  }
}

