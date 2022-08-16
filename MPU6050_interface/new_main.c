/*
 * mpu6050_interface.c
 *
 * Created: 05-08-2022 19:59:03
 * Author : anany
 */ 

 #define F_CPU 16000000UL

#include <inttypes.h>
#include <stdint.h>
#include <avr/io.h>
#include <util/delay.h>
#include <util/setbaud.h>
#include <avr/interrupt.h>
#include <math.h>
#include "mpu6050.h"
#include "mpu6050_reg.h"
#include "i2c.h"
#include "uart.h"


int main(void){

sei();  
uart_init();
i2c_init();

DDRB |= _BV(5);

uint8_t ret;

int16_t accel_buff[3], gyro_buff[3];
double accelX, accelY, accelZ;
double gyroX, gyroY, gyroZ;
double biasX, biasY;
double phi_accel, theta_accel;
double phi_innov, theta_innov;
double phi_est, theta_est;
double phi_prev, theta_prev;

char s[30];

// initialize & test MPU5060 availability
ret = i2c_start(MPU6050_ADDRESS+I2C_WRITE);
if(~ret){
	PORTB |= _BV(5);
	_delay_ms(200);
	PORTB &= ~(_BV(5));
}

mpu6050_init();

// find gyro bias
biasX = 0;
biasY = 0;
uint8_t i;
for(i=0; i<20; i++){
	mpu6050_read_gyro_ALL(gyro_buff);
	biasX += gyro_buff[0];
	biasY += gyro_buff[1];
	/*
	PORTB |= _BV(5);
	_delay_ms(20);
	PORTB &= ~(_BV(5));
	*/
}
biasX = biasX/20*(3.14159/180)/1000/32768;
biasY = biasY/20*(3.14159/180)/1000/32768;

// initialization for Kalman filter
double P = 0.0;
double Q = 0.001;
double R = 0.03;
double Pp, K;
mpu6050_read_accel_ALL(accel_buff);
phi_prev = atan2(accelY, accelZ); // row
theta_prev = atan2(-accelX, sqrt(accelY*accelY+accelZ*accelZ)); // pitch

for(;;){
	get_time(&dt);
	mpu6050_read_accel_ALL(accel_buff);
	mpu6050_read_gyro_ALL(gyro_buff);
	
	// acceleration (m/s^2)
	accelX = accel_buff[0]*9.8*2/32768;
	accelY = accel_buff[1]*9.8*2/32768;
	accelZ = accel_buff[2]*9.8*2/32768;
	// gyro rate (rad/s)
	gyroX = gyro_buff[0]*(3.14159/180)/1000/32768;
	gyroY = gyro_buff[1]*(3.14159/180)/1000/32768;
	gyroZ = gyro_buff[2]*(3.14159/180)/1000/32768;

	// estimation
	phi_est = phi_prev + dt*(gyroX - biasX);
	theta_est = theta_prev + dt*(gyroY - biasY);
	Pp = P+Q;

	// innovation
	phi_accel = atan2(accelY, accelZ); // row
	phi_innov = phi_accel - phi_est;
	theta_accel = atan2(-accelX, sqrt(accelY*accelY+accelZ*accelZ)); // pitch
	theta_innov = theta_accel - theta_est;

	// Kalman gain
	K = Pp/(Pp+R);

	// correction
	phi_prev = phi_prev + K*phi_innov;
	theta_prev = theta_prev + K*theta_innov;
	P = (1-K)*Pp;
	
	uart_putchar('\n');
	_delay_ms(10);
	uart_putdouble(phi_prev); //phi, row
	uart_putdouble(theta_prev); //theta, pitch	
	//_delay_ms(10);
		
}

}//end of main


