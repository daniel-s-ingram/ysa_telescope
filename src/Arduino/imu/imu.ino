#include "Wire.h"

#define MPU9255_ADDRESS 0x68
#define AK8963_ADDRESS 0x0C
#define WHO_AM_I 0x75
#define INT_PIN_CFG 0x37
#define MAG_CNTL 0x0A
#define ACCEL_XOUT 0x3B
#define ACCEL_YOUT 0x3D
#define ACCEL_ZOUT 0x3F
#define GYRO_XOUT 0x43
#define GYRO_YOUT 0x45
#define GYRO_ZOUT 0x47
#define MAG_XOUT 0x03
#define MAG_YOUT 0x05
#define MAG_ZOUT 0x07

const double pi = 3.14159265358979;
const double gyro_deg2rad = 500.0 * pi / 5898240.0;

int i,j;

int32_t a[3];
int32_t g[3];
int32_t m[3];

byte device_id;
byte error;
byte acc_address[] = {ACCEL_XOUT, ACCEL_YOUT, ACCEL_ZOUT};
byte gyro_address[] = {GYRO_XOUT, GYRO_YOUT, GYRO_ZOUT};
byte mag_address[] = {MAG_XOUT, MAG_YOUT, MAG_ZOUT};

double roll, pitch, rollRate, pitchRate, rollAngle, pitchAngle, dt, rollGyro;

void setup() 
{
  Wire.begin();
  
  Serial.begin(115200);
  Serial.println("Initializing I2C communication with MPU9255...");
  //Check that the device ID stored in the WHO_AM_I register matches the known ID of 0x71 for the MPU9255
  //This is to ensure that the device is properly connected
  device_id = read(MPU9255_ADDRESS, WHO_AM_I, 1);
  Serial.println(device_id == 0x71 ? "Communication with MPU9255 successful" : "MPU9255 not found");
  while(!(device_id == 0x71));

  //Set to pass-through mode to allow reading directly from magnetometer
  write(MPU9255_ADDRESS, INT_PIN_CFG, 0x02);
  write(AK8963_ADDRESS, MAG_CNTL, 0x16);

  dt = micros();
}
void loop() 
{
  for(i = 0;i < 3;i++){
   a[i] = read(MPU9255_ADDRESS, acc_address[i], 2);
   g[i] = read(MPU9255_ADDRESS, gyro_address[i], 2);
  }

  magRead();
  
  //Convert accelerometer data to degrees and gyroscope data to degrees/second
  pitch = atan2(-a[1], a[2]); 
  roll = atan2(-a[0], a[2]);
  pitchRate = g[0] * gyro_deg2rad;
  rollRate = g[1] * gyro_deg2rad;
  
  //Complementary filter
  rollAngle = 0.99 * (rollAngle + rollRate * (micros() - dt) / 1000000.0) + 0.01 * roll;
  pitchAngle = 0.98 * (pitchAngle + pitchRate * (micros() - dt) / 1000000.0) + 0.02 * pitch;
  
  dt = micros();

  Serial.print(m[0]); Serial.print(" "); Serial.print(m[1]); Serial.print(" "); Serial.println(m[2]);
  //Serial.print(rollAngle); Serial.print(" "); Serial.println(pitchAngle);
}
int16_t read(byte deviceAddress, byte valueAddress, int bytesToRead)
{
  //Specify the register address from which to read
  Wire.beginTransmission(deviceAddress);
  Wire.write(valueAddress);
  error = Wire.endTransmission();
  
  //Read value from specified register
  //When reading the device ID, only one read needs to be performed as the ID is one byte
  //The acc and gyro values are two bytes with the high and low bytes alternating e.g. ACC_XOUT_H, ACC_XOUT_L, ACC_YOUT_H, etc
  //Since the high byte is read first, the value is bit shifted right 8 times before being added to the low byte
  Wire.beginTransmission(deviceAddress);
  Wire.requestFrom(deviceAddress, bytesToRead);
  
  if(bytesToRead == 1){
   return Wire.read();
  }
  else{
   return (Wire.read() << 8) + Wire.read();
  }
  
  error = Wire.endTransmission();
}

void magRead()
{
  for (int i = 0; i < 3; i++)
  {
    m[i] = read(AK8963_ADDRESS, mag_address[i], 2);
  }
  read(AK8963_ADDRESS, 0x09, 1);
}

void write(byte deviceAddress, byte valueAddress, byte data)
{
  Wire.beginTransmission(deviceAddress);
  Wire.write(valueAddress);
  Wire.write(data);
  Wire.endTransmission();
}

