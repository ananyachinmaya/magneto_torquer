/*
 * mpu6050_interface.c
 *
 * Created: 05-08-2022 19:59:03
 * Author : anany
 */ 

#include <avr/io.h>
#define F_CPU 16000000UL
#define FOSC 16000000UL
#define BAUD 9600UL
#define BAUDRATE (FOSC/(16*BAUD) - 1)
#include <inttypes.h>
#include <stdint.h>
#include <util/setbaud.h>
#include <avr/interrupt.h>
#include <math.h>
#include "mpu6050.h"
#include "mpu6050_reg.h"
#include "i2c.h"
#include "uart.h"
#include <util/delay.h>
void register_address(int x){
	while (!(TWCR & (1<<TWINT)));
	TWDR = x;
}
//function to configure registers in mpu6050
void register_config(int x){
	while (!(TWCR & (1<<TWINT)));
	TWDR = x;
}
//function to acquire data from mpu6050
int register_data(int x){
	register_address(x);
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
	while(!((TWCR) & (1<<TWINT)));
	return TWDR;
}
/*
   UART functions to transmit data from microcontroller to realterm
*/
//Initialising function
void USART_Init( unsigned int ubrr )
{
	UBRR0H = (unsigned char)(ubrr>>8);   //BITSHIFT for 8 msb's
	UBRR0L = (unsigned char) ubrr;
	UCSR0B = (1<<RXEN)|(1<<TXEN);       //enables transmitter and receiver
	UCSR0C = (1<<USBS0)|(3<<UCSZ0);     //usbs -> no.of stop bits , ucsz -> no. of bits shared
}
//Transmit function
void USART_Transmit( int data )
{
	while ( !( UCSR0A & (1<<UDRE)) );
	UDR0 = data;
}
int main(void)
{	
	double biasX, biasY, accx_h, accx_l, accy_h, accy_l, accz_h, accz_l, accx, accy, accz;
	int16_t accel_buff[3], gyro_buff[3];
	double accelX, accelY, accelZ;
	double gyroX, gyroY, gyroZ;
	double phi_accel, theta_accel;
	double phi_innov, theta_innov;
	double phi_est, theta_est;
	double phi_prev, theta_prev;
	sei();
	i2c_init();
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

    while (1) 
    {
		//GETTING DATA FROM MPU6050	
			i2c_start(MPU6050_ADDRESS + I2C_WRITE);
			
			while(!((TWSR & 0xF8) == 0x08));
			TWDR = 0x68 ;
			TWCR = (1<<TWINT)|(1<<TWEN);
			register_address(0x1B);
			TWCR = (1<<TWINT)|(1<<TWEN);
			register_config(0xFF);
			TWCR = (1<<TWINT)|(1<<TWEN);
			register_address(0x1C);
			TWCR = (1<<TWINT)|(1<<TWEN);
			register_config(0xFF);
			/* getting data of the measurements of accelerometer in
			 x, y and z */			
			 accx = (accx_h>>8)|(accx_l);
			 accy = (accy_h>>8)|(accy_l);
			accz = (accz_h>>8)|(accz_l);
			/* getting data of the measurements of gyroscope in
			 x, y and z */
			/*int gyrox_h = register_data(67);
			int gyrox_l = register_data(68);
			int gyroy_h = register_data(69);
			int gyroy_l = register_data(70);
			int gyroz_h = register_data(71);
			int gyroz_l = register_data(72);*/
			i2c_stop();
		//Transmitting data to realterm via UART
			USART_Transmit(accx);
			_delay_ms(50);
			USART_Transmit(accy);
			_delay_ms(50);
			USART_Transmit(accz);
			_delay_ms(50);
    }
}

