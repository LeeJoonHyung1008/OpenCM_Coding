/*
*  SPI communication.
*  Master version.
*  created 27 Aug 2017.
*  by Jinwoo Choo, Chung-nam national univ.
*/

HardwareSPI spi(1);

unsigned int cnt=0;
unsigned int input=0;

void setup() {
  spi.begin(SPI_281_250KHZ, MSBFIRST, 0);
}

void loop() {
  /*if(SerialUSB.available()){
    cnt = SerialUSB.read();
    SerialUSB.print(cnt);
  spi.write(cnt);
  }*/
  input = spi.send(cnt);
  SerialUSB.print("Received data : ");
  SerialUSB.print(input);
  SerialUSB.print("\tsent data : ");
  SerialUSB.println(cnt);
  cnt+=5;
  /*if(cnt>=255){
    cnt=0;
  }*/
  delay(1000);
}
