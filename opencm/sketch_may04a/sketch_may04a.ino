#include <math.h>
#include <Wire.h>
#define step_MX64
#define D2P_ (4096/360)

#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04
#define DXL_BUS_SERIAL2 2  //Dynamixel on Serial2(USART2)  <-LN101,BT210
#define DXL_BUS_SERIAL3 3  //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP

Dynamixel Dxl1(DXL_BUS_SERIAL3);  // MX-64

int SensorPin = 0;

byte x = 0;
byte y = 0;
int flag_2 = 0;

byte moving_status_1, moving_status_2, moving_status_4  = 0;
byte read_status_1, read_status_2, read_status_4 = 0;
/*void HLbyte(int input, byte *x, byte *y)
{
 (byte) x = input / 256;
 (byte) y = input % 256;
}*/

//int buffer_z[3];               //통신을 할때 buffer배열에 전송받은 데이터 입력
double l1=33, l2=26;
double A, B, theta1, theta2;

float mea_z;
int flag;
 
int atime_final[4];               //통신을 할때 buffer배열에 전송받은 데이터 입력
int apresent_position_x[4];               //통신을 할때 buffer배열에 전송받은 데이터 입력
int agoal_position_x[4];               //통신을 할때 buffer배열에 전송받은 데이터 입력
int adt_position_x[4];               //통신을 할때 buffer배열에 전송받은 데이터 입력

int apresent_position_y[4];               //통신을 할때 buffer배열에 전송받은 데이터 입력
int agoal_position_y[4];               //통신을 할때 buffer배열에 전송받은 데이터 입력
int adt_position_y[4];               //통신을 할때 buffer배열에 전송받은 데이터 입력

int apresent_position_z[4];               //통신을 할때 buffer배열에 전송받은 데이터 입력
int agoal_position_z[4];               //통신을 할때 buffer배열에 전송받은 데이터 입력
int adt_position_z[4];               //통신을 할때 buffer배열에 전송받은 데이터 입력
 
float a_x, b_x, c_x = 0;
float Position_x, Velocity_x, Acceleration_x;
float a_y, b_y, c_y = 0;
float Position_y, Velocity_y, Acceleration_y;
float a_z, b_z, c_z = 0;
float Position_z, Velocity_z, Acceleration_z;

float dt = 0;
float temp1, temp2, temp3, temp4 = 0;

byte Moving_Status1;
byte Moving_Status2;

//  double z;
  double time_final, goal_position_x, dt_position_x, init_position_x; 
  double goal_position_y, dt_position_y, init_position_y; 
  double goal_position_z, dt_position_z, init_position_z; 
  
byte ismoving123;

void setup() {

  // Dynamixel 2.0 Baudrate -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps 

  //Serial.begin(1000000);
  Wire.begin(14,15);        // join i2c bus (SDA 14 SCL 15)
  SerialUSB.begin();

  Dxl1.begin(3);
  Dxl1.setPacketType(DXL_PACKET_TYPE2);
  Dxl1.writeByte(2, 11, 3);    
  Dxl1.writeByte(2, 64, 1);  //MX-64 1,2,4를 torque enable 해준다 
  Dxl1.writeByte(1, 11, 3);
  Dxl1.writeByte(1, 64, 1);
  Dxl1.writeByte(4, 11, 3);
  Dxl1.writeByte(4, 64, 1);

  //Step Profile 
  Dxl1.writeDword(1, 112, 10);
  Dxl1.writeDword(2, 112, 10);
  Dxl1.writeDword(4, 112, 10);
  
  /*ismoving123 = Dxl1.readDword(1,112);
  SerialUSB.println(ismoving123);
  ismoving123 = Dxl1.readDword(2,112);
  SerialUSB.println(ismoving123);
  ismoving123 = Dxl1.readDword(4,112);
  SerialUSB.println(ismoving123);*/

  temp1 = Dxl1.readDword(1, 132); 
  temp2 = Dxl1.readDword(2, 132); 
  temp4 = Dxl1.readDword(4, 132);
  
  moving_status_1 = Dxl1.readByte(1, 123);
  moving_status_1 &= 0b11001111;
  moving_status_2 = Dxl1.readByte(2, 123);
  moving_status_2 &= 0b11001111;
  moving_status_4 = Dxl1.readByte(4, 123);
  moving_status_4 &= 0b11001111;
  
  Dxl1.writeByte(1, 123, moving_status_1); 
  Dxl1.writeByte(2, 123, moving_status_2);
  Dxl1.writeByte(4, 123, moving_status_4);
  
  read_status_1 = Dxl1.readByte(1, 123);
  read_status_2 = Dxl1.readByte(2, 123);
  read_status_4 = Dxl1.readByte(4, 123);
  SerialUSB.println(read_status_1);
  SerialUSB.println(read_status_2);
  SerialUSB.println(read_status_4);

  SerialUSB.print("motor1 current encoder :  ");
  SerialUSB.println(temp1);
  SerialUSB.print("motor2 current encoder :  ");
  SerialUSB.println(temp2);
  SerialUSB.print("motor4 current encoder :  ");
  SerialUSB.println(temp4);

    Dxl1.writeDword(1, 116, 2048); //Compatible with all dynamixel serise
    Dxl1.writeDword(2, 116, 2048); //Compatible with all dynamixel serise
    Dxl1.writeDword(4, 116, 316); //Compatible with all dynamixel serise

  init_position_x = -26;
  init_position_y = 33;
  
}

void loop() {  
  temp1 = Dxl1.readDword(1, 132); 
  temp2 = Dxl1.readDword(2, 132); 
  temp4 = Dxl1.readDword(4, 132); 
  
  SerialUSB.print("motor1 current encoder :  ");
  SerialUSB.println(temp1);
  SerialUSB.print("motor2 current encoder :  ");
  SerialUSB.println(temp2);
  SerialUSB.print("motor4 current encoder :  ");
  SerialUSB.println(temp4);
  SerialUSB.println(read_status_1);
  SerialUSB.println(read_status_2);
  SerialUSB.println(read_status_4);
  
  if(SerialUSB.available()) {
    if(SerialUSB.read()=='s') {
      if(flag_2 % 2 == 0)      // even, odd      %n n번에 1번씩 해당 코드를 실행 
        {
         x = 3;
         y = 141; 
        }
      else{
       x=2;
       y=68;
       }
      Wire.beginTransmission(4); // transmit to device #4  slave id : 4
      Wire.write(x);              // sends one byte  
      Wire.write(y);              // sends one byte  
      Wire.endTransmission();    // stop transmitting

      SerialUSB.println(x * 256 + y);
      flag_2++;
    }
    else
    {
     SerialUSB.print("length of 1st link : ");
     SerialUSB.println(l1);
     SerialUSB.print("length of 2nd link :  ");
     SerialUSB.println(l2);

     if(SerialUSB.available() ) {
        atime_final[0]  = SerialUSB.read() - 48;   //시리얼 통신으로 버퍼배열에 데이터 수신
        atime_final[1]  = SerialUSB.read() - 48;
        atime_final[2]  = SerialUSB.read() - 48;
      } 
      time_final = (atime_final[0] * 100 + atime_final[1] * 10 + atime_final[2]);
      SerialUSB.print("Enter 'time final' values.");
      SerialUSB.println(time_final);
      
      if(SerialUSB.available() ) {
        agoal_position_x[0]  = SerialUSB.read();   //시리얼 통신으로 버퍼배열에 데이터 수신
        agoal_position_x[1]  = SerialUSB.read() - 48;
        agoal_position_x[2]  = SerialUSB.read() - 48;
        agoal_position_x[3]  = SerialUSB.read() - 48;
      } 
      if(agoal_position_x[0]==45) {
        goal_position_x = - (agoal_position_x[1]*100 + agoal_position_x[2]*10 + agoal_position_x[3]);
      }
      else {
        goal_position_x = agoal_position_x[1]*100 + agoal_position_x[2]*10 + agoal_position_x[3];
      }
      SerialUSB.print("Enter 'goal position_x' values.");
      SerialUSB.println(goal_position_x);
    
      if(SerialUSB.available() ) {
        agoal_position_y[0]  = SerialUSB.read();   //시리얼 통신으로 버퍼배열에 데이터 수신
        agoal_position_y[1]  = SerialUSB.read() - 48;
        agoal_position_y[2]  = SerialUSB.read() - 48;
        agoal_position_y[3]  = SerialUSB.read() - 48;
      } 
      if(agoal_position_y[0]==45) {
        goal_position_y = - (agoal_position_y[1]*100 + agoal_position_y[2]*10 + agoal_position_y[3]);
      }
      else {
        goal_position_y = agoal_position_y[1]*100 + agoal_position_y[2]*10 + agoal_position_y[3];
      }
     SerialUSB.print("Enter 'goal position_y' values.");
     SerialUSB.println(goal_position_y);
  
     if(SerialUSB.available() ) {
        agoal_position_z[0]  = SerialUSB.read();   //시리얼 통신으로 버퍼배열에 데이터 수신
        agoal_position_z[1]  = SerialUSB.read() - 48;
        agoal_position_z[2]  = SerialUSB.read() - 48;
        agoal_position_z[3]  = SerialUSB.read() - 48;
      } 
      if(agoal_position_z[0]==45) {
        goal_position_z = - (agoal_position_z[1]*100 + agoal_position_z[2]*10 + agoal_position_z[3]);
      }
      else {
        goal_position_z = agoal_position_z[1]*100 + agoal_position_z[2]*10 + agoal_position_z[3];
      }
      SerialUSB.print("Enter 'goal position_z' values");
      SerialUSB.println(goal_position_z);                    //int값으로 변환된 데이터 출력
      SerialUSB.print("'init position_x' values.");
      SerialUSB.println(init_position_x);
      SerialUSB.print("'init position_y' values.");
      SerialUSB.println(init_position_y);
      SerialUSB.print("'init position_z' values.");
      SerialUSB.println(init_position_z);
      SerialUSB.println("=============================================");

     /// Calculate coefficient 
     a_x = 6 / pow(time_final, 5) * (goal_position_x - init_position_x);
     b_x = -15 / pow(time_final, 4) * (goal_position_x - init_position_x);
     c_x = 10 / pow(time_final, 3) * (goal_position_x - init_position_x);

     a_y = 6 / pow(time_final, 5) * (goal_position_y - init_position_y);
     b_y = -15 / pow(time_final, 4) * (goal_position_y - init_position_y);
     c_y = 10 / pow(time_final, 3) * (goal_position_y - init_position_y);
   
     a_z = 6 / pow(time_final, 5) * (goal_position_z - init_position_z);
     b_z = -15 / pow(time_final, 4) * (goal_position_z - init_position_z);
     c_z = 10 / pow(time_final, 3) * (goal_position_z - init_position_z);
   
     flag = 100; 
    }
  }
  
//  mea_z = z * 10.0 / 14 * 180 / PI * D2P_;    // calculate AX-12A moving

   if(flag == 100){
      for (dt = 0; dt < time_final; )
      {  
       /// Calculate instance position X,Y
       dt_position_x = a_x * pow(dt, 5) + b_x * pow(dt, 4) + c_x * pow(dt, 3) + init_position_x;
       //SerialUSB.print("Enter 'dt_position_x' values.");
       //SerialUSB.println(dt_position_x);
       Velocity_x = 5 * a_x * pow(dt, 4) + 4 * b_x * pow(dt, 3) + 3 * c_x * pow(dt, 2);
       Velocity_x = Velocity_x * 0.682;
       Acceleration_x = 20 * a_x*pow(dt, 3) + 12 * b_x * pow(dt, 2) + 6 * c_x * dt;
       Acceleration_x = Acceleration_x * 40.9255568;


       dt_position_y = a_y * pow(dt, 5) + b_y * pow(dt, 4) + c_y * pow(dt, 3) + init_position_y;
       //SerialUSB.print("Enter 'dt_position_y' values.");
       //SerialUSB.println(dt_position_y);
       Velocity_y = -(5 * a_y * pow(dt, 4) + 4 * b_y * pow(dt, 3) + 3 * c_y * pow(dt, 2));
       Velocity_y = Velocity_y * 0.682;
       Acceleration_y = -(20 * a_y*pow(dt, 3) + 12 * b_y * pow(dt, 2) + 6 * c_y * dt);
       Acceleration_y = Acceleration_y * 40.9255568;
       
       
       dt_position_z = a_z * pow(dt, 5) + b_z * pow(dt, 4) + c_z * pow(dt, 3) + init_position_z;
       //SerialUSB.print("Enter 'dt_position_y' values.");
       //SerialUSB.println(dt_position_y);
       Velocity_z = -(5 * a_z * pow(dt, 4) + 4 * b_z * pow(dt, 3) + 3 * c_z * pow(dt, 2));
       Velocity_z = Velocity_z * 0.682; 
       Acceleration_z = -(20 * a_z*pow(dt, 3) + 12 * b_z * pow(dt, 2) + 6 * c_z * dt);
       Acceleration_z = Acceleration_z * 40.9255568;
       
       mea_z = dt_position_z * 10.0 / 14 * 180 / PI * D2P_;
       
       /// Calculate theta 
       theta2 = acos((((dt_position_x * dt_position_x) + (dt_position_y * dt_position_y) - (l1 * l1) - (l2 * l2))) / (2 * (l1 * l2))) ;
       theta1 = atan2(dt_position_y, dt_position_x) - asin((l2 * sin(theta2)) / sqrt((dt_position_x * dt_position_x) + (dt_position_y * dt_position_y)) ) ;
       theta2 *= 180/PI;
       theta1 *= 180/PI;
       SerialUSB.println(dt);
       SerialUSB.print("\t");
       SerialUSB.println(theta1);
       SerialUSB.print("\t\t");
//       SerialUSB.print("theta2 is ");
       SerialUSB.println(theta2);
       theta1 = 2048 + (theta1 - 90) * D2P_;
       theta2 = 2048 + (theta2 - 90)  * D2P_;

       Dxl1.writeDword(1, 116, theta1);      
       delay(10);
//       Dxl1.writeDword(1, 112, Velocity_x);
//       Dxl1.writeDword(1, 108, Acceleration_x);

       
       Dxl1.writeDword(2, 116, theta2);
       delay(10);
//       Dxl1.writeDword(2, 112, Velocity_y);
//       Dxl1.writeDword(2, 108, Acceleration_y);

     
       Dxl1.writeDword(4, 116, mea_z);
       delay(10);
//       Dxl1.writeDword(4, 112, Velocity_z);
//       Dxl1.writeDword(4, 108, Acceleration_z);


/*       SerialUSB.print("motor1 current encoder :  ");
       SerialUSB.println(temp1);
       SerialUSB.print("motor2 current encoder :  ");
       SerialUSB.println(temp2);
       SerialUSB.print("motor3 current encoder :  ");
       SerialUSB.println(temp3);
       SerialUSB.print("motor4 current encoder :  ");
       SerialUSB.println(temp4);    */

       dt += 0.1;
       } // end of for()
       flag = 0;
       
       // 3*256 + 16 = 784 = 0x0310
     
//       Dxl.writeWord(3, 32, 50);    
//       Dxl.writeWord(3, 30, 1023);    
       
       init_position_x = dt_position_x;
       init_position_y = dt_position_y;
       init_position_z = dt_position_z;
       
  }

}
