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
<<<<<<< Updated upstream
FusionAhrs ahrs;
Adafruit_BMP3XX bmp;
Adafruit_ISM330DHCX ism330dhcx;
//File flight_data;
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
float elapsed_launch=0;
float elapsed_parachute=0;
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
  //SD.begin(BUILTIN_SDCARD);
  //flight_data = SD.open("flight_data.csv");
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
=======
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
Servo parachute;
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
  parachute.attach(TVCPpin);
>>>>>>> Stashed changes
}

void loop() {
  update_sensors();
  launchdetect();
  if (az_g<0.8 & (prev_altitude - altitude)>0){
    elapsed_parachute += sampletime;
  }
  if(elapsed_parachute > 0.3){
    Parachute.write(180); //or 0, depending on the orientation of parachute ejection mechanism
    state=2;
  }
  
  //Serial.println((String)pitch+" "+yaw+" "+pwmy+" "+pwmx);
  Serial.println((String)ax_g+" "+ay_g+" "+az_g+" "+state+" "+elapsed_launch+" "+elapsed_parachute);
}

void launchdetect(){
  if(az_g >= 1.15){
    elapsed_launch += sampletime;
  }
  if(elapsed_launch > 0.15){
    state = 1;
  }
  if(state ==1){
    pid();
  }
}

//tuneable values
float kp = 3.55;  
float kd = 2.05;
//float ki = 2.05;

float ratiox = 1;
float ratioy = 1;
float servo_offsetx = 90;
float servo_offsety = 90;

float setpointx=0;
float setpointy=0;
float errorx=0;
float errory=0;
float PIDX,PIDY,lastErrx,lastErry;
void pid(){
  lastErrx = errorx;
  lastErry = errory;
  errorx = pitch-setpointx;
  errory = yaw-setpointy; 
  
  float dErrx = (errorx - lastErrx)/sampletime;
  PIDX = kp*errorx+kd*dErrx;
  float dErry = (errory - lastErry)/sampletime;
  PIDY = kp*errory+kd*dErry;
  
  pwmx = servo_offsetx + PIDX*ratiox; //can be plus or minus depending on whether which side is 0 or 180
  pwmy = servo_offsety + PIDY*ratioy;
  TVCX.write(pwmx);
  TVCY.write(pwmy);
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
  g_pitch = gyro.gyro.y+0.05;   //in rad/s
  g_yaw = gyro.gyro.z;
  ax_g = accel.acceleration.z/9.81; 
  ay_g = accel.acceleration.y/9.81;    //in gs
  az_g = accel.acceleration.x/9.81; 
  //madgwick filter 
  madgwick_update(ax_g,ay_g,az_g,g_pitch,g_yaw,g_roll);
}

void madgwick_update(float ax, float ay, float az, float gx, float gy, float gz){
  FusionVector gyroscope = {gx, gy, gz}; 
  FusionVector accelerometer = {ax, ay, az}; 
  FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer,0.0384615);
  
  const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
  yaw = euler.angle.roll;
  pitch = euler.angle.pitch;
  roll = euler.angle.yaw;
}