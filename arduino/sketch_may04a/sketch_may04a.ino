#include <Wire.h> 
#include <AX12A.h>
#include <string.h>

#define DirectionPin (10u)
#define BaudRate (1000000ul)
#define ID (3u)

byte DATA[2];
int Ax_joint;

byte State[2];

void setup()
{
  Wire.begin(4);                 // set up i2c  slave id : 4
  Wire.onReceive(receiveEvent);   // attach receive event similar interrupt  INT
  Wire.onRequest(sendToMaster);
  Serial.begin(1000000);             // init Serial device
  delay(1000);
  ax12a.begin(BaudRate, DirectionPin, &Serial); // motor 설정
  ax12a.moveSpeed(ID, 0, 100);
}

void loop() // LED blinking
{
  Ax_joint = ((DATA[0] * 256) + DATA[1]);
  ax12a.move(ID, Ax_joint);
}

void sendToMaster()
{
  byte read_State[2] ={0, };
  int Pre_Position = ax12a.readPosition(ID);
  
  read_State[0] = Pre_Position % 256;
  read_State[1] = Pre_Position / 256; 
  Wire.write(read_State[1]);
  Wire.write(read_State[0]);
}
// i2c receiving event when data coming from master device(OpenCM9.04)
void receiveEvent(int howMany)
{
   while( Wire.available())   // when data is available
   {
      DATA[0] = Wire.read();     // read data
      DATA[1] = Wire.read();     // read data
   } 
}
