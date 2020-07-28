#include <HardwareSPI.h>
/*
*  SPI communication.
*  Slave version.
*  created 27 Aug 2017.
*  by Jinwoo Choo, Chung-nam national univ.
*/

HardwareSPI spi(1);

int data; 
int olddata;

void setup() {
  spi.beginSlave(MSBFIRST, 0);
}

void loop() {
  data = spi.read();
  if(olddata==data){  
    data = spi.read();
  }
  SerialUSB.print("data : ");
  SerialUSB.println(data,DEC);
  olddata=data;
}
