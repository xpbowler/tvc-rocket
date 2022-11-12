#include <Wire.h>
#include <Servo.h>
#include "I2Cdev.h"
#include <BMP388_DEV.h>
#include <Adafruit_ISM330DHCX.h>
#include <SPI.h>
#include <SP.h>
#include <math.h>
#include <stdio.h>
#define SEALEVELPRESSURE_HPA (1013.25)

//Useful functions
//tone(output pin, tone, duration)
//servo.write(theta). theta is degrees. make sure also to add delay(500) or something after
//digitalWrite(ledpin,(LOW or HIGH)). low is off. high is on.

//global variables
float ax_g,ay_g,az_g,gx_rad,gy_rad,gz_rad,pressure,altitude,temperature;
float roll=0.0, pitch=0.0, yaw=0.0;
float orientation[2] = {pitch,yaw};
struct quaternion q_est = { 1, 0, 0, 0};       // initialize with as unit vector with real component  = 1
double currentTime, prevTime, dt;
int sampletime;
File flight_data;

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

void setup() {
  // setup code
  serial.begin(115200);
  //Wire.begin();
  //Wire.setClock(400000UL);

  //SD read/write
  SD.begin(BUILTIN_SDCARD);

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

  //TVC servos
  TVCX.attach(TVCXpin);
  TVCY.attach(TVCYpin);
  Parachute.attach(Parachutepin);

  startup();
}

void loop() {
  // main code, to run repeatedly:
  prevTime = currentTime;
  currentTime = millis();
  sampletime = (currentTime - prevTime)/1000; //in seconds

  update_sensors();
  launchdetect();
  datastore();
  if (az_g<5){
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

void launchdetect(){
  if(az_g >= 5){
    state = 1;
  }
  if(state ==1){
    pid();
  }
}

double initial_land;
double final_land;
void land_detect(){
  if(ax_g<0.5 & altitude<2){
    initial_land = currentTime;
    final_land = currentTime;
  }
}

void datastore(){
  flight_data = SD.open("flight_data.txt");
  flight_data.println(currentTime+" "+roll+" "+yaw+" "+pitch+" "+temperature
  +" "+altitude+" "+pressure+" "+ax_g+" "+ay_g+" "+az_g+" "+gx_rad+" "+state);
}

int sx_start = 80; //tbd
int sy_start = 80; //tbd
void startup () {
  digitalWrite(G_LED, HIGH);
  TVCX.write(sx_start);
  TVCY.write(sy_start);
  delay(1000);
  digitalWrite(G_LED, LOW);
  digitalWrite(Buzzer, HIGH);
  digitalWrite(R_LED, HIGH);
  TVCX.write(sx_start + 20);
  delay(200);
  digitalWrite(Buzzer, LOW);
  TVCY.write(sy_start + 20);
  delay(200);
  digitalWrite(R_LED, LOW);
  digitalWrite(Buzzer, HIGH);
  digitalWrite(B_LED, HIGH);
  TVCX.write(sx_start);
  delay(200);
  digitalWrite(Buzzer, LOW);
  TVCY.write(sy_start);
  delay(200);
  digitalWrite(B_LED, LOW);
  digitalWrite(Buzzer, HIGH);
  digitalWrite(G_LED, HIGH);
  TVCX.write(sx_start - 20);
  delay(200);
  digitalWrite(Buzzer, LOW);
  TVCY.write(sy_start - 20);
  delay(200);
  TVCX.write(sx_start);
  delay(200);
  TVCY.write(sy_start);
  delay(500);
 }

void update_sensors(void){
  sensors_event_t accel;
  sensors_event_t gyro;
  //update sensor values
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

// Include a hardware specific header file to redefine these predetermined values
float DELTA_T = 0.01f; // 100Hz sampling frequency
float PI = 3.14159265358979f;
float GYRO_MEAN_ERROR = PI * (5.0f / 180.0f); // 5 deg/s gyroscope measurement error (in rad/s)  *from paper*
float BETA = sqrt(3.0f/4.0f) * GYRO_MEAN_ERROR;    //*from paper*

/*struct quaternion{
  float q1;
  float q2;
  float q3;
  float q4;
};
*/

// Multiply a reference of a quaternion by a scalar, q = s*q
void quat_scalar(struct quaternion * q, float scalar){
  q -> q1 *= scalar;
  q -> q2 *= scalar;
  q -> q3 *= scalar;
  q -> q4 *= scalar;
}

// Adds two quaternions together and the sum is the pointer to another quaternion, Sum = L + R
void quat_add(struct quaternion * Sum, struct quaternion L, struct quaternion R){
  Sum -> q1 = L.q1 + R.q1;
  Sum -> q2 = L.q2 + R.q2;
  Sum -> q3 = L.q3 + R.q3;
  Sum -> q4 = L.q4 + R.q4;
}

// Subtracts two quaternions together and the sum is the pointer to another quaternion, sum = L - R
void quat_sub(struct quaternion * Sum, struct quaternion L, struct quaternion R){
  Sum -> q1 = L.q1 - R.q1;
  Sum -> q2 = L.q2 - R.q2;
  Sum -> q3 = L.q3 - R.q3;
  Sum -> q4 = L.q4 - R.q4;
}

// the conjugate of a quaternion is it's imaginary component sign changed  q* = [s, -v] if q = [s, v]
struct quaternion quat_conjugate(struct quaternion q){
  q.q2 = -q.q2;
  q.q3 = -q.q3;
  q.q4 = -q.q4;
  return q;
}

float quat_Norm (struct quaternion q)
{
  return sqrt(q.q1*q.q1 + q.q2*q.q2 + q.q3*q.q3 +q.q4*q.q4);
}

// Normalizes pointer q by calling quat_Norm(q),
void quat_Normalization(struct quaternion * q){
  float norm = quat_Norm(*q);
  q -> q1 /= norm;
  q -> q2 /= norm;
  q -> q3 /= norm;
  q -> q4 /= norm;
}

void printQuaternion (struct quaternion q){
  printf("%f %f %f %f\n", q.q1, q.q2, q.q3, q.q4);
}