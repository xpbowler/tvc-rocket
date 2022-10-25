#include <stdio.h>
#include "madgwickFilter.h"
#include "madgwick.c"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h> 
#include <SPIFlash.h>
#include <Servo.h>
#include "I2Cdev.h"
#include <Adafruit_ISM330DHCX.h>

//global variables
float ax_g,ay_g,az_g,gx_rad,gy_rad,gz_rad,pressure,altitude,temperature;
int pflight;
int R_LED = 38;
int G_LED = 34;
int B_LED = 35;
int Buzzer = 28;
int Pyro1 = 22;
int Pyro2 = 2;
int Pyro3 = 29;
int Pyro4 = 37;
int TVCXpin = 9;
int TVCYpin = 15;
Servo TVCXservo;
Servo TVCYservo;
Adafruit_BMP280 bmp;
Adafruit_ISM330DHCX imu;
int16_t ax, ay, az;
int16_t gx, gy, gz;
#define OUTPUT_READABLE_IMU

#define CHIPSIZE MB64
SPIFlash flash(1);
uint8_t pageBuffer[256];
String serialCommand;
char printBuffer[128];
uint16_t page;
uint8_t offset, dataByte;
uint16_t dataInt;
String inputString, outputString;

void setup() {
  // setup code
  serial.begin(115200);
  pinMode(R_LED, OUTPUT);
  pinMode(G_LED, OUTPUT);
  pinMode(B_LED, OUTPUT);
  pinMode(Pyro1, OUTPUT);
  pinMode(Pyro2, OUTPUT);
  pinMode(Pyro3, OUTPUT);
  pinMode(Pyro4, OUTPUT);
  if(!bmp.begin()){
    //barometer configuration
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  }
  //IMU configuration
  imu.initialize();
  imu.setAccelRange(LSM6DS_ACCEL_RANGE_4_G); //set accel range (in gs)
  imu.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS); //set gryo range (in degrees/s)
  imu.setAccelDataRate(LSM6DS_RATE_12_5_HZ); 
  imu.setGyroDataRate(LSM6DS_RATE_12_5_HZ); 
  imu.configInt1(false, false, true); // accelerometer DRDY on INT1 (dont know what these lines do)
  imu.configInt2(false, true, false); // gyro DRDY on INT2
  //TVC servos
  TVCXservo.attach(TVCXpin);
  TVCYservo.attach(TVCYpin);
}
void loop() {
  // main code, to run repeatedly:
  update_sensors();
  if (ax_g >= 6){
     pflight = 1;
  }
  if (pflight == 1){
    tvc();
  }
}

void tvc(){
    

}
void update_sensors(void){
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  imu.getEvent(&accel, &gyro, &temp);
  //update sensor values
  gx_rad = gyro.gyro.x;
  gy_rad = gyro.gyro.y;   //in rad/s
  gz_rad = gyro.gyro.z;
  ax_g = accel.acceleration.x / 9.81;
  ay_g = accel.acceleration.y / 9.81;   //in gs, or is m/s^2 fine?
  az_g = accel.acceleration.z / 9.81; 
  temperature = temp.temperature;
  pressure = bmp.readPressure();    //in Pa
  altitude = bmp.readAltitude(1013.25); //needs to be adjusted to local forecast. in meters
  //madgwick filter 
  float roll=0.0, pitch=0.0, yaw=0.0;
  madgwick_update(ax_g,ay_g,az_g,gx_rad,gy_rad,gz_rad);
  eulerAngles(q_est,&roll,&pitch,&yaw);
}