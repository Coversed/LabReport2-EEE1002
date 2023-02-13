#include <Wire.h>
#define I2C_SLAVE_ADDR 0x04 // 4 in hexadecimal
// Arduino Nano Basic Line Following Robot Code
// Code hosted on https://github.com/Mcspanky93/Line-Follower-Calibration-Buzzer
void Transmit_to_arduino(int leftMotor_speed, int rightMotor_speed, int servoAngle)
{
  Wire.beginTransmission(I2C_SLAVE_ADDR); // transmit to device #4
  Wire.write((byte)((leftMotor_speed & 0x0000FF00) >> 8));    // first byte of x, containing bits 16 to 9
  Wire.write((byte)(leftMotor_speed & 0x000000FF));           // second byte of x, containing the 8 LSB - bits 8 to 1
  Wire.write((byte)((rightMotor_speed & 0x0000FF00) >> 8));   // first byte of y, containing bits 16 to 9
  Wire.write((byte)(rightMotor_speed & 0x000000FF));          // second byte of y, containing the 8 LSB - bits 8 to 1
  Wire.write((byte)((servoAngle & 0x0000FF00) >> 8));    // first byte of x, containing bits 16 to 9
  Wire.write((byte)(servoAngle & 0x000000FF));
  Wire.endTransmission();   // stop transmitting
}

int sensor1= 32; //connected to analog 0  Yellow, R
int sensor2= 35; // connected to analog 1 Green, SR
int sensor3= 33; // connected to analog 2 Purple, MR
int sensor4= 25; // connected to analog 3 Pink, ML
int sensor5= 34; // connected to analog 4 Orange, SL
int sensor6= 26; // connected to analog 5 FR Blue, L Broke
int SenVal1 = 0;
int SenVal2 = 0;
int SenVal3 = 0;
int SenVal4 = 0;
int SenVal5 = 0;
int SenVal6 = 0;
int Val1 = 0;
int Val2 = 0;
int Val3 = 0;
int Val4 = 0;
int Val5 = 0;
int Val6 = 0;
float Xpk = 0;
float error = 0;
float Kp = 3;
float Ki = 0.3;
float Kd = 0.6;
float cumulativeerror = 0;
float previouserror = 0;
float servoAngle = 0;
float u = 0;
float K = 0.5;
float Ku = 0;

void setup()
{
  Wire.begin();
  Serial.begin(9600);
  SenVal1 =  analogRead(sensor1);
  SenVal2  = analogRead(sensor2);
  SenVal3  = analogRead(sensor3);
  SenVal4  = analogRead(sensor4);
  SenVal5  = analogRead(sensor5);
  SenVal6  = analogRead(sensor6);
  
}

void loop(){        
// Read sensor's collected values:
  SenVal1 =  analogRead(sensor1);
  SenVal2  = analogRead(sensor2);
  SenVal3  = analogRead(sensor3);
  SenVal4  = analogRead(sensor4);
  SenVal5  = analogRead(sensor5);
  SenVal6  = analogRead(sensor6);
  Val1 = map(SenVal1,870,4095,180,0);
  Val2 = map(SenVal2,820,4095,180,0);
  Val3 = map(SenVal3,980,4095,180,0);
  Val4 = map(SenVal4,990,4095,180,0);
  Val5 = map(SenVal5,560,4095,180,0);
  Val6 = map(SenVal6,0,4095,180,0); //Replace
// Weighted average 
  Xpk = ((((Val1*40)+(Val2*25)+(Val3*10)+(Val4*-25)+Val5*-40)+(Val6*0)) / (Val1 + Val2 + Val3 + Val4 + Val5 + Val6)); //Replace Val6 multiplier
  error = Xpk;
  u = Kp*error+Ki*cumulativeerror+Kd*(error-previouserror);
  Serial.println(u);  
  servoAngle = 90+u;
  Ku = K*u;
  cumulativeerror = cumulativeerror+error;
  previouserror = error;
  Transmit_to_arduino(-90+Ku,-90+Ku,servoAngle);

  // Delay statement, experimentally 50ms anything less results in the non communication
  delay(50);
}