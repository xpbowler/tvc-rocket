#include <math.h>
#include <stdio.h>
#include "Fusion.h"

#include <stdbool.h>
#include "I2Cdev.h"
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_ISM330DHCX.h>
#define SAMPLE_RATE(12.5) //using imu refresh rate = 12.5Hz
#include "madgwickfilter.h"


//global variables
float ax_g,ay_g,az_g,gx_rad,gy_rad,gz_rad,pressure,altitude,temperature, prev_altitude;
float roll=0.0, pitch=0.0, yaw=0.0;
float orientation[2] = {pitch,yaw};
double currentTime, prevTime, dt;
int sampletime;
File flight_data;
int sx_start = 80; //tbd
int sy_start = 80; //tbd
float orientationx = pitch;
float orientationy = yaw;
struct quaternion q_est = { 1, 0, 0, 0};       // initialize with as unit vector with real component  = 1
float currentTime, prevTime, dt;
float sampletime;
File flight_data;
int SEALEVELPRESSURE_HPA = 1013.25;

float ax, ay, az;
float gx, gy, gz;
Adafruit_BMP3XX bmp;
Adafruit_ISM330DHCX ism330dhcx;

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

// Define calibration (replace with actual calibration data)
const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
const FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};
const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
const FusionVector accelerometerOffset = {0.0f, 0.0f, 0.0f};
const FusionMatrix softIronMatrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector hardIronOffset = {0.0f, 0.0f, 0.0f};

// Initialise algorithms
FusionOffset offset;
FusionAhrs ahrs;
FusionOffsetInitialise(&offset, SAMPLE_RATE);
FusionAhrsInitialise(&ahrs);
// Set AHRS algorithm settings
const FusionAhrsSettings settings = {
  .gain = 0.5f,
  .accelerationRejection = 10.0f,
  .magneticRejection = 20.0f,
  .rejectionTimeout = 5 * SAMPLE_RATE, /* 5 seconds */
};

FusionAhrsSetSettings(&ahrs, &settings);}
void setup() {
  serial.begin(115200);
void setup(void) {
  Serial.begin(115200);
  //Wire.begin();
  //Wire.setClock(400000UL); . dont know if needed

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
  ism330dhcx.setAccelRange(LSM6DS_ACCEL_RANGE_4_G); //set accel range (in gs)
  ism330dhcx.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS); //set gryo range (in degrees/s)
  ism330dhcx.setAccelDataRate(LSM6DS_RATE_12_5_HZ); 
  ism330dhcx.setGyroDataRate(LSM6DS_RATE_12_5_HZ); 
  ism330dhcx.configInt1(false, false, true); // accelerometer DRDY on INT1 (dont know what these lines do)
  ism330dhcx.configInt2(false, true, false); // gyro DRDY on INT2

  //servos
  TVCX.attach(TVCXpin);
  TVCY.attach(TVCYpin);
  Parachute.attach(Parachutepin);

  if(ism330dhcx.begin_I2C() & bmp388.begin()){
    digitalWrite(G_LED,HIGH);
    digitalWrite(Buzzer, HIGH);
    delay(1500);
    digitalWrite(Buzzer,LOW);

  }
}

double elapsed_parachute = 0;
void loop() {
  prevTime = currentTime;
  currentTime = millis();
  sampletime = (currentTime - prevTime)/1000; //in seconds. currentTime and prevTime are in milliseconds. 

  update_sensors();
  launchdetect();
  datastore();
  Serial.println((String)roll+" "+pitch+" "+yaw);
  if (az_g<5 & (prev_altitude - altitude)>0){
    elapsed_parachute += sampletime;
  }
  if(elapsed_parachute > 0.4){
    delay(4000);
    Parachute.write(90);
    state = 2;
  }
  if(state ==2){
    land_detect();
  }
}

float kp = 3.55;  //kp,kd,ki need to be tuned for optimal PID control
float kd = 2.05;
float ki = 2.05;
float divisor;
float setpointx = 0;
float setpointy = 0;
float errSumx = 0;
float errSumy = 0;
float lastErrx = 0;
float lastErry = 0;
float correction_anglex = 0;
float correction_angley = 0;
float pwmx = 0;  //x is first, y is second
float pwmy = 0;
float servo_gear_ratio = 5.8; //change according to our servo
float servo_offsetx = 100;
float servo_offsety = 100;
void pid(){
  float errorx = orientationx - setpointx;
  errSumx += errorx * sampletime;
  float dErrx = (errorx - lastErrx)/sampletime;
  correction_anglex = (kp*errorx + ki*errSumx + kd*dErrx)/divisor;
  lastErrx = errorx;
  
  float errory = orientationy - setpointy;
  errSumy += errory * sampletime;
  float dErry = (errory - lastErry)/sampletime;
  correction_angley = (kp*errory + ki*errSumy + kd*dErry)/divisor;
  lastErry = errory;
  pwmx = ((correction_anglex * servo_gear_ratio) + servo_offsetx);
  pwmy = ((correction_angley * servo_gear_ratio) + servo_offsety);
  TVCX.write(pwmx);
  TVCY.write(pwmy);
}

double elapsed_launch = 0;
void launchdetect(){
  if(az_g >= 5){
    elapsed_launch += sampletime;
  }
  if(elapsed_launch > 0.4){
    state = 1;
  }
  if(state ==1){
    pid();
  }
}

double elapsed_land = 0;
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
}

void datastore(){
  flight_data.println((String)gx_rad+" "+gy_rad+" "+gz_rad+" "+ax_g+" "+ay_g+" "+az_g+" "+gx_rad);
}

void update_sensors(void){
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  //update sensor values
  prev_altitude = altitude;
  temperature = bmp.temperature;
  pressure = bmp.pressure;
  altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA); //in C,hpa,m
  ism330dhcx.getEvent(&accel, &gyro, &temp); 
  gx_rad = gyro.gyro.x;
  gy_rad = gyro.gyro.y;   //in rad/s
  gz_rad = gyro.gyro.z;
  ax_g = accel.acceleration.x; 
  ay_g = accel.acceleration.y;    //in gs, or is m/s^2 fine?
  az_g = accel.acceleration.z; 
  //madgwick filter 
  madgwick_update(ax_g,ay_g,az_g,gx_rad,gy_rad,gz_rad);
}

void madgwick_update(float ax, float ay, float az, float gx, float gy, float gz){
  // Acquire latest sensor data
  const clock_t timestamp = millis()); // replace this with actual gyroscope timestamp
  FusionVector gyroscope = {gx, gy, gz}; // replace this with actual gyroscope data in degrees/s
  FusionVector accelerometer = {ax, ay, az}; // replace this with actual accelerometer data in g

  // Apply calibration
  gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
  accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);

  // Update gyroscope offset correction algorithm
  gyroscope = FusionOffsetUpdate(&offset, gyroscope);

  // Calculate delta time (in seconds) to account for gyroscope sample clock error
  static clock_t previousTimestamp;
  const float deltaTime = (float) (timestamp - previousTimestamp) / (float) CLOCKS_PER_SEC;
  previousTimestamp = timestamp;

  // Update gyroscope AHRS algorithm
  FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, deltaTime);

  // Print algorithm outputs
  const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
  const FusionVector earth = FusionAhrsGetEarthAcceleration(&ahrs);


}