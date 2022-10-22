#include <stdio.h>
#include "madgwickFilter.h"
#include "madgwick.c"

#define MPU6050_ADDRESS  0b1101000
#define HMC5883L_ADDRESS 0b0011110
#define MS5611_ADDRESS   0b1110111

//global variables
float ax_g,ay_g,az_g,gx_rad,gy_rad,gz_rad,pressure,altitude;
int pflight;

void setup() {
    // setup code, to run once:

    // configure i2c
    i2c_setup(I2C1, FAST_MODE_400KHZ, PB8, PB9);
    // configure the MPU6050 (gyro/accelerometer)
    i2c_write_register(I2C1, MPU6050_ADDRESS,  0x6B, 0x00);                    // exit sleep
	i2c_write_register(I2C1, MPU6050_ADDRESS,  0x19, 109);                     // sample rate = 8kHz / 110 = 72.7Hz
	i2c_write_register(I2C1, MPU6050_ADDRESS,  0x1B, 0x18);                    // gyro full scale = +/- 2000dps
	i2c_write_register(I2C1, MPU6050_ADDRESS,  0x1C, 0x08);                    // accelerometer full scale = +/- 4g
	i2c_write_register(I2C1, MPU6050_ADDRESS,  0x38, 0x01);                    // enable INTA interrupt
    
    // configure an external interrupt for the MPU6050's active-high INTA signal
	gpio_setup(PB7, INPUT, PUSH_PULL, FIFTY_MHZ, NO_PULL, AF0);
	exti_setup(PB7, RISING_EDGE);
}

void loop() {
  // main code, to run repeatedly:
  update_orientation();
  if (az >= 6){
     pflight = 1;
  }
  if (pflight == 1){
    tvc();
  }
}

void tvc(){
    

}
void update_orientation(void){
    //update sensor values
    uint8_t rx_buffer[20];
	i2c_read_registers(I2C1, MPU6050_ADDRESS, 20, 0x3B, rx_buffer);
    int ax = rx_buffer[0];
    int ay = rx_buffer[2];
    int az = rx_buffer[4];
    int gx = rx_buffer[6];
    int gy = rx_buffer[8];
    int gz = rx_buffer[10];

    // convert accelerometer readings into G's
	ax_g = ax / 8192.0f;
	ay_g = ay / 8192.0f;
	az_g = az / 8192.0f;
    //convert gyroscope readings into rad/s
    gx_rad = gx / 939.650784f;
	gy_rad = gy / 939.650784f;
	gz_rad = gz / 939.650784f;


    //madgwick filter 
    float roll=0.0, pitch=0.0, yaw=0.0;
    madgwick_update(ax_g,ay_g,az_g,gx_rad,gy_rad,gz_rad);
    eulerAngles(q_est,&roll,&pitch,&yaw);
}