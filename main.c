#include <math.h>
#include <stdio.h>
#include "Fusion.h"
#include <stdbool.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_ISM330DHCX.h>


//global variables
FusionAhrs ahrs;
Adafruit_BMP3XX bmp;
Adafruit_ISM330DHCX ism330dhcx;
File flight_data;
float ax_g,ay_g,az_g,g_yaw,g_roll,g_pitch,pressure,altitude,temperature,prev_altitude,
currentTime,prevTime,dt,sampletime;
int SEALEVELPRESSURE_HPA = 1013.25;
float roll=0.0;
float pitch=0.0;
float yaw=0.0;
float pwmx = 0;  
float pwmy = 0;
int TVCXpin = 33;
int TVCYpin = 29;
int Parachutepin = 0;
Servo TVCX;
Servo TVCY;
Servo Parachute;
int state = 0;
int R_LED = 37;
int G_LED = 13;
int B_LED = 14;
int Buzzer = 36;
int sx_start = 80; //tbd
int sy_start = 80; //tbd

void setup(void) {
  FusionAhrsInitialise(&ahrs);
  Serial.begin(115200);
  //SD read/write
  SD.begin(BUILTIN_SDCARD);
  flight_data = SD.open("flight_data.csv");
  //led
  pinMode(R_LED, OUTPUT);
  pinMode(G_LED, OUTPUT);
  pinMode(B_LED, OUTPUT);
  //barometer
  bmp.begin_I2C();
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  //imu
  ism330dhcx.begin_I2C();
  ism330dhcx.setAccelRange(LSM6DS_ACCEL_RANGE_4_G); //set accel range 
  ism330dhcx.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS); //set gyro range
  ism330dhcx.setAccelDataRate(LSM6DS_RATE_26_HZ); 
  ism330dhcx.setGyroDataRate(LSM6DS_RATE_26_HZ); 
  ism330dhcx.configInt1(false, false, true); // accelerometer DRDY on INT1 
  ism330dhcx.configInt2(false, true, false); // gyro DRDY on INT2
  //servos
  TVCX.attach(TVCXpin);
  TVCY.attach(TVCYpin);
  Parachute.attach(Parachutepin);
}

void loop() {
  update_sensors();
  datastore();
  pid();
  Serial.println((String)yaw+" "+pitch+" "+pwmx+" "+pwmy);
  //Serial.println((String)az_g+" "+ay_g+" "+ax_g+" "+g_roll+" "+g_yaw+" "+g_pitch);
}

//tuneable values
float kp = 3.55;  
float kd = 2.05;
//float ki = 2.05;
float servo_offsetx = 90;
float servo_offsety = 90;
float ratiox = 2;
float ratioy = 2;

float setpointx = 0;
float setpointy = 0;
float lastErrx = 0;
float lastErry = 0;
float PIDX = 0;
float PIDY = 0;
void pid(){
  lastErrx = errorx;
  lastErry = errory;
  float errorx = pitch-setpointx;
  float errory = yaw-setpointy; 
  
  float dErrx = (errorx - lastErrx)/sampletime;
  PIDX = kp*errorx+kd*dErrx;
  
  float dErry = (errory - lastErry)/sampletime;
  PIDY = kp*errory+kd*dErry;

  pwmx = servo_offsetx + PIDX*ratiox; //can be plus or minus depending on whether which side is 0 or 180
  pwmy = servo_offsety + PIDY*ratioy;
  TVCX.write(pwmx);
  TVCY.write(pwmy);
}

double elapsed_launch = 0;
void launchdetect(){
  pid();
}
/*double elapsed_land = 0;
void land_detect(){
  if(ax_g<0.5 & altitude<2){
    elapsed_land += sampletime;
  }
  if(elapsed_land>=6){
    flight_data.close();
    TVCX.write(sx_start);
    TVCY.write(sy_start);
    //add whatever else we want to do after the rocket lands here
    exit(0); //does this end the program?
  }
}*/

void datastore(){
  //flight_data.println((String)yaw+" "+pitch+" "+roll+" "+altitude+" "+pressure);
}

void update_sensors(void){
  prevTime = currentTime;
  currentTime = millis();
  sampletime = (currentTime - prevTime)/1000; //in seconds. currentTime and prevTime are in milliseconds. 
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  //update sensor values
  prev_altitude = altitude;
  temperature = bmp.temperature;
  pressure = bmp.pressure;
  altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA); //in C,hpa,m
  ism330dhcx.getEvent(&accel, &gyro, &temp); 
  g_roll = gyro.gyro.x;
  g_pitch = gyro.gyro.y;   //in rad/s
  g_yaw = gyro.gyro.z;
  ax_g = accel.acceleration.z/9.81; 
  ay_g = accel.acceleration.y/9.81;    //in gs
  az_g = accel.acceleration.x/9.81; 
  //madgwick filter 
  madgwick_update(ax_g,ay_g,az_g,g_roll,g_pitch,g_yaw);
}

void madgwick_update(float ax, float ay, float az, float gx, float gz, float gy){
  // Acquire latest sensor data
  FusionVector gyroscope = {gx, gy, gz}; // replace this with actual gyroscope data in degrees/s
  FusionVector accelerometer = {ax, ay, az}; // replace this with actual accelerometer data in g
  // Update gyroscope AHRS algorithm
  FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer,0.0384615);
  // Print algorithm outputs
  const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
  roll = euler.angle.roll;
  pitch = euler.angle.pitch;
  yaw = euler.angle.yaw;
}