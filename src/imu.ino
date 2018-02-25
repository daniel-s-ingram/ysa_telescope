#include "Wire.h"

#define MPU9255_ADDRESS 0x68
#define WHO_AM_I 0x75
#define ACCEL_XOUT 0x3B
#define ACCEL_YOUT 0x3D
#define ACCEL_ZOUT 0x3F
#define GYRO_XOUT 0x43
#define GYRO_YOUT 0x45
#define GYRO_ZOUT 0x47

const double pi = 3.14159265358979;

int i,j;

int32_t a[3];
int32_t g[3];
int32_t m[3];

byte device_id;
byte error;
byte acc_address[] = {ACCEL_XOUT, ACCEL_YOUT, ACCEL_ZOUT};
byte gyro_address[] = {GYRO_XOUT, GYRO_YOUT, GYRO_ZOUT};

double roll, pitch, rollRate, pitchRate, rollAngle, pitchAngle, dt, rollGyro;

void setup() {
  Wire.begin();
  
  Serial.begin(115200);
  Serial.println("Initializing I2C communication with MPU9255...");
  //Check that the device ID stored in the WHO_AM_I register matches the known ID of 0x73 for the MPU9255
  //This is to ensure that the device is properly connected
  device_id = read(WHO_AM_I, 1);
  Serial.println(device_id == 0x68 ? "Communication with MPU9255 successful" : "MPU9255 not found");
  
//  while(!(device_id == 0x68)); //Don't continue to loop() if ID check fails
//
//  Wire.beginTransmission(MPU9255_ADDRESS);
//  Wire.write(0x6B);
//  Wire.write(0x00);
//  error = Wire.endTransmission();
//
//  Wire.beginTransmission(MPU9255_ADDRESS);
//  Wire.write(0x6C);
//  Wire.write(0x00);
//  error = Wire.endTransmission();

  dt = micros();
  rollAngle = 0;
  pitchAngle = 0;
  rollGyro = 0;
}
void loop() {
  for(i = 0;i < 3;i++){
   a[i] = read(acc_address[i], 2);
   g[i] = read(gyro_address[i], 2);
  }
  //Serial.print(a[0]); Serial.print(" "); Serial.print(a[1]); Serial.print(" "); Serial.println(a[2]);
  //Convert accelerometer data to degrees and gyroscope data to degrees/second
  pitch = atan2(-a[1], a[2]) * 180.0 / pi; 
  roll = atan2(-a[0], a[2]) * 180.0 / pi;
  pitchRate = g[0] * 500.0 / 32768.0;
  rollRate = g[1] * 500.0 / 32768.0;
  //Serial.print(pitch); Serial.print(" "); //Serial.println(roll);
  
  //Complementary filter
  rollAngle = 0.99 * (rollAngle + rollRate * (micros() - dt) / 1000000.0) + 0.01 * roll;
  pitchAngle = 0.98 * (pitchAngle + pitchRate * (micros() - dt) / 1000000.0) + 0.02 * pitch;

  rollGyro = rollGyro + rollRate * (micros() - dt) / 1000000.0;
  
  dt = micros();

  /*int turnLR,turnUD;
  if(rollRate > 10) turnLR = 1;
  else if (rollRate < -10) turnLR = -1;
  else turnLR = 0;

  if(pitchRate > 10) turnUD = 1;
  else if (pitchRate < -10) turnUD = -1;
  else turnUD = 0;  
  
  Serial.print(turnLR); Serial.print(","); Serial.println(turnUD);*/
  
  Serial.print(rollAngle); Serial.print(" "); Serial.println(pitchAngle); //Serial.print(" "); 
  //Serial.print(roll); Serial.print(" "); 
  //Serial.print(rollGyro); Serial.print(" "); 
  //Serial.println(rollAngle);
  //Serial.print(a[0]); Serial.print(" "); Serial.print(a[2]); Serial.print(" "); Serial.print(a[2]*a[2] + a[0]*a[0]); Serial.print(" "); Serial.println(sqrt(a[2]*a[2] + a[0]*a[0]));

  delay(50);
}
int16_t read(byte valueAddress, int bytesToRead){
  //Specify the register address from which to read
  Wire.beginTransmission(MPU9255_ADDRESS);
  Wire.write(valueAddress);
  error = Wire.endTransmission();
  
  //Read value from specified register
  //When reading the device ID, only one read needs to be performed as the ID is one byte
  //The acc and gyro values are two bytes with the high and low bytes alternating e.g. ACC_XOUT_H, ACC_XOUT_L, ACC_YOUT_H, etc
  //Since the high byte is read first, the value is bit shifted right 8 times before being added to the low byte
  Wire.beginTransmission(MPU9255_ADDRESS);
  Wire.requestFrom(MPU9255_ADDRESS, bytesToRead);
  
  if(bytesToRead == 1){
   return Wire.read();
  }
  else{
   return (Wire.read() << 8) + Wire.read();
  }
  
  error = Wire.endTransmission();
}
