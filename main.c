#include <math.h>
#include <stdio.h>
#include "Fusion.h"

#include <stdbool.h>
#include "I2Cdev.h"
#include <SPI.h>
#include <SP.h>
#include <Wire.h>
#include <Servo.h>
#include <BMP388_DEV.h>
#include <Adafruit_ISM330DHCX.h>
#define SAMPLE_RATE(12.5) //using imu refresh rate = 12.5Hz

//global variables
float ax_g,ay_g,az_g,gx_rad,gy_rad,gz_rad,pressure,altitude,temperature, prev_altitude;
float roll=0.0, pitch=0.0, yaw=0.0;
float orientation[2] = {pitch,yaw};
double currentTime, prevTime, dt;
int sampletime;
File flight_data;
int sx_start = 80; //tbd
int sy_start = 80; //tbd

int ax, ay, az;
int gx, gy, gz;
BMP388_DEV bmp;
Adafruit_ISM330DHCX imu;

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
  //Wire.begin();
  //Wire.setClock(400000UL); . dont know if needed

  //SD read/write
  SD.begin(BUILTIN_SDCARD);
  flight_data = SD.open("flight_data.txt");

  //led
  pinMode(R_LED, OUTPUT);
  pinMode(G_LED, OUTPUT);
  pinMode(B_LED, OUTPUT);

  //barometer
  bmp388.begin(SLEEP_MODE, OVERSAMPLING_X16, OVERSAMPLING_X2, IIR_FILTER_4, TIME_STANDBY_5MS);
  bmp388.startNormalConversion();

  //imu
  ism330dhcx.begin_I2C();
  imu.initialize();
  imu.setAccelRange(LSM6DS_ACCEL_RANGE_4_G); //set accel range (in gs)
  imu.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS); //set gryo range (in degrees/s)
  imu.setAccelDataRate(LSM6DS_RATE_12_5_HZ); 
  imu.setGyroDataRate(LSM6DS_RATE_12_5_HZ); 
  imu.configInt1(false, false, true); // accelerometer DRDY on INT1 (dont know what these lines do)
  imu.configInt2(false, true, false); // gyro DRDY on INT2

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
double divisor;
float setpoint[2] = {0,0};
double errSum[2] = {0,0};
float lastErr[2] = {0,0};
double correction_angle[2]; 
void pid(){
  float error[2] = orientation - setpoint;
  errSum += error * sampletime;
  double dErr[2] = (error - lastErr)/sampletime;
  correction_angle = (kp*error + ki*errSum + kd*dErr)/divisor;
  lastErr = error;
  servo_actuation(correction_angle);
}

double pwm[2] = {0,0};  //x is first, y is second
double servo_gear_ratio = 5.8; //change according to our servo
double servo_offset[2] = {100,100}; //change according to our servo
void servo_actuation(correction_angle){
  pwm = ((correction_angle * servo_gear_ratio) + servo_offset);
  TVCX.write(pwm[1]);
  TVCY.write(pwm[2]);
  //dont rlly know about this shit 
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
  flight_data.println(currentTime+" "+roll+" "+yaw+" "+pitch+" "+temperature
  +" "+altitude+" "+pressure+" "+ax_g+" "+ay_g+" "+az_g+" "+gx_rad+" "+state);
}

void update_sensors(void){
  sensors_event_t accel;
  sensors_event_t gyro;
  //update sensor values
  prev_altitude = altitude;
  bmp.getMeasurements(temperature,pressure,altitude);    //in C,hPa,m
  imu.getEvent(&accel, &gyro); 
  gx_rad = gyro.gyro.x;
  gy_rad = gyro.gyro.y;   //in rad/s
  gz_rad = gyro.gyro.z;
  ax_g = accel.acceleration.x / 9.81;
  ay_g = accel.acceleration.y / 9.81;   //in gs, or is m/s^2 fine?
  az_g = accel.acceleration.z / 9.81; 
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