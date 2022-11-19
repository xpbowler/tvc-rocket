#include <math.h>
#include <stdio.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_ISM330DHCX.h>
#include "madgwickfilter.h"


//global variables
float ax_g,ay_g,az_g,gx_rad,gy_rad,gz_rad,pressure,altitude,temperature;
float prev_altitude;
float roll=0.0, pitch=0.0, yaw=0.0;
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
  eulerAngles(q_est,&roll,&pitch,&yaw);
}

// Multiply two quaternions and return a copy of the result, prod = L * R
struct quaternion quat_mult (struct quaternion L, struct quaternion R){
    
    
    struct quaternion product;
    product.q1 = (L.q1 * R.q1) - (L.q2 * R.q2) - (L.q3 * R.q3) - (L.q4 * R.q4);
    product.q2 = (L.q1 * R.q2) + (L.q2 * R.q1) + (L.q3 * R.q4) - (L.q4 * R.q3);
    product.q3 = (L.q1 * R.q3) - (L.q2 * R.q4) + (L.q3 * R.q1) + (L.q4 * R.q2);
    product.q4 = (L.q1 * R.q4) + (L.q2 * R.q3) - (L.q3 * R.q2) + (L.q4 * R.q1);
    
    return product;
}

// Updates q_est. gyro angular velocity components are in rad/s, accelerometer components are normalized
void madgwick_update(float ax, float ay, float az, float gx, float gy, float gz){
    //Variables and constants
    struct quaternion q_est_prev = q_est;
    struct quaternion q_est_dot = {0};           
    const struct quaternion q_g_ref = {0, 0, 0, 1};
    struct quaternion q_a = {0, ax, ay, az};    // raw acceleration values (not normalized)
    float F_g [3] = {0};                        // objective function for gravity
    float J_g [3][4] = {0};                     // jacobian matrix
    struct quaternion gradient = {0};           // gradient 
    
    // Attitude estimate from gyro with numerical integration 
    struct quaternion q_w;                   // places gyroscope readings in a quaternion
    q_w.q1 = 0;                              // real component 0
    q_w.q2 = gx;
    q_w.q3 = gy;
    q_w.q4 = gz;
    quat_scalar(&q_w, 0.5);                  
    q_w = quat_mult(q_est_prev, q_w);        // dq/dt = (1/2)q*w

    //Attitude estimate from acc with gradient descent 
    quat_Normalization(&q_a);              // normalize acc quaternion
    F_g[0] = 2*(q_est_prev.q2 * q_est_prev.q4 - q_est_prev.q1 * q_est_prev.q3) - q_a.q2;    //Compute objective function for gravity
    F_g[1] = 2*(q_est_prev.q1 * q_est_prev.q2 + q_est_prev.q3* q_est_prev.q4) - q_a.q3;
    F_g[2] = 2*(0.5 - q_est_prev.q2 * q_est_prev.q2 - q_est_prev.q3 * q_est_prev.q3) - q_a.q4;
    
    //Compute jacobian matrix
    J_g[0][0] = -2 * q_est_prev.q3;
    J_g[0][1] = 2 * q_est_prev.q4;
    J_g[0][2] = -2 * q_est_prev.q1;
    J_g[0][3] = 2 * q_est_prev.q2;
    J_g[1][0] = 2 * q_est_prev.q2;
    J_g[1][1] = 2 * q_est_prev.q1;
    J_g[1][2] = 2 * q_est_prev.q4;
    J_g[1][3] = 2 * q_est_prev.q3;
    J_g[2][0] = 0;
    J_g[2][1] = -4 * q_est_prev.q2;
    J_g[2][2] = -4 * q_est_prev.q3;
    J_g[2][3] = 0;
    
    // compute gradient = J_g'*F_g
    gradient.q1 = J_g[0][0] * F_g[0] + J_g[1][0] * F_g[1] + J_g[2][0] * F_g[2];
    gradient.q2 = J_g[0][1] * F_g[0] + J_g[1][1] * F_g[1] + J_g[2][1] * F_g[2];
    gradient.q3 = J_g[0][2] * F_g[0] + J_g[1][2] * F_g[1] + J_g[2][2] * F_g[2];
    gradient.q4 = J_g[0][3] * F_g[0] + J_g[1][3] * F_g[1] + J_g[2][3] * F_g[2];
    
    // gradient normalization
    quat_Normalization(&gradient);
  
    //sensor fusion
    quat_scalar(&gradient, BETA);             // multiply normalized gradient by beta
    quat_sub(&q_est_dot, q_w, gradient);        // subtract above from q_w, the integrated gyro quaternion
    quat_scalar(&q_est_dot, DELTA_T);
    quat_add(&q_est, q_est_prev, q_est_dot);     // Integrate orientation rate to find position
    quat_Normalization(&q_est);                 // normalize the orientation of the estimate
}

//returns as pointers, roll pitch and yaw from q, using right hand system (Roll,x,phi)(Pitch,y,theta)(Yaw,z,psi) 
void eulerAngles(struct quaternion q, float* roll, float* pitch, float* yaw){
    
    *yaw = atan2f((2*q.q2*q.q3 - 2*q.q1*q.q4), (2*q.q1*q.q1 + 2*q.q2*q.q2 -1));  
    *pitch = -asinf(2*q.q2*q.q4 + 2*q.q1*q.q3);                                  
    *roll  = atan2f((2*q.q3*q.q4 - 2*q.q1*q.q2), (2*q.q1*q.q1 + 2*q.q4*q.q4 -1));
    
    *yaw *= (180.0f / PI);
    *pitch *= (180.0f / PI);
    *roll *= (180.0f / PI);

}