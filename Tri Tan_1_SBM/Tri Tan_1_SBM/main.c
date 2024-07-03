#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <inttypes.h>
#include "I2C_Master_H_file.h"
#include "USART_RS232_H_file.h"

/* Define motor control pins */
#define ENA PB3
#define IN1 PD6
#define IN2 PD5
#define IN3 PB1
#define IN4 PB2
#define ENB PD3

/* PID constants */
#define Kp 1.0
#define Ki 0.0
#define Kd 0.0

/* MPU6050 I2C address */
#define MPU6050_ADDR 0x68

/* MPU6050 register addresses */
#define MPU6050_RA_PWR_MGMT_1 0x6B
#define MPU6050_RA_SMPLRT_DIV 0x19
#define MPU6050_RA_CONFIG 0x1A
#define MPU6050_RA_GYRO_CONFIG 0x1B
#define MPU6050_RA_ACCEL_CONFIG 0x1C
#define MPU6050_RA_ACCEL_XOUT_H 0x3B

/* Function prototypes */
void MPU6050_Init();
void MPU6050_Calibrate(int16_t* gyroX_offset, int16_t* gyroY_offset, int16_t* gyroZ_offset);
void MPU6050_Read_RawData(int16_t* AccelX, int16_t* AccelY, int16_t* AccelZ, int16_t* GyroX, int16_t* GyroY, int16_t* GyroZ);
void PWM_Init();
void Motor_SetSpeed(uint8_t left_speed, uint8_t right_speed);
void Motor_SetDirection(bool forward);
void Motor_Stop();
float PID_Controller(float setpoint, float measured, float* previous_error, float* integral);

/* Global variables for PID control */
float previous_error = 0.0;
float integral = 0.0;

int main(void)
{
	int16_t AccelX, AccelY, AccelZ, GyroX, GyroY, GyroZ;
	int16_t gyroX_offset = 0, gyroY_offset = 0, gyroZ_offset = 0;
	float angle = 0.0, setpoint = 0.0;
	uint8_t left_pwm_value = 255;
	uint8_t right_pwm_value = 255;

	/* Initialize modules */
	I2C_Init();
	MPU6050_Init();
	MPU6050_Calibrate(&g yroX_offset, &gyroY_offset, &gyroZ_offset);
	PWM_Init();
	USART_Init(9600);

	/* Initial motor speed */
	Motor_SetSpeed(left_pwm_value, right_pwm_value);
	Motor_SetDirection(true);

	while(1)
	{
		/* Read raw data from MPU6050 */
		MPU6050_Read_RawData(&AccelX, &AccelY, &AccelZ, &GyroX, &GyroY, &GyroZ);

		/* Apply gyro offsets */
		GyroX -= gyroX_offset;
		GyroY -= gyroY_offset;
		GyroZ -= gyroZ_offset;

		/* Calculate angle using complementary filter */
		angle = 0.98 * (angle + (GyroX / 131.0) * 0.01) + 0.02 * (atan2(AccelY, AccelZ) * 180 / M_PI);

		/* Calculate PID output */
		float pid_output = PID_Controller(setpoint, angle, &previous_error, &integral);

		/* Adjust motor speed based on PID output */
		if(pid_output > 0)
		{
			Motor_SetDirection(true);
			left_pwm_value = 200 + (uint8_t)pid_output;
			right_pwm_value = 200 - (uint8_t)pid_output;
		}
		else
		{
			Motor_SetDirection(true);
			left_pwm_value = 200 - (uint8_t)(-pid_output);
			right_pwm_value = 200 + (uint8_t)(-pid_output);
		}

		Motor_SetSpeed(left_pwm_value, right_pwm_value);
		_delay_ms(10);
	}
}

/* MPU6050 functions */
void MPU6050_Init()
{
	I2C_Start_Wait(MPU6050_ADDR << 1);
	I2C_Write(MPU6050_RA_PWR_MGMT_1);
	I2C_Write(0x00); // Wake up MPU6050
	I2C_Stop();

	I2C_Start_Wait(MPU6050_ADDR << 1);
	I2C_Write(MPU6050_RA_SMPLRT_DIV);
	I2C_Write(0x07); // Set sample rate to 1kHz
	I2C_Stop();

	I2C_Start_Wait(MPU6050_ADDR << 1);
	I2C_Write(MPU6050_RA_CONFIG);
	I2C_Write(0x00); // Set DLPF to 260Hz
	I2C_Stop();

	I2C_Start_Wait(MPU6050_ADDR << 1);
	I2C_Write(MPU6050_RA_GYRO_CONFIG);
	I2C_Write(0x18); // Set gyro range to +/- 2000 deg/sec
	I2C_Stop();

	I2C_Start_Wait(MPU6050_ADDR << 1);
	I2C_Write(MPU6050_RA_ACCEL_CONFIG);
	I2C_Write(0x01); // Set accelerometer range to +/- 2g
	I2C_Stop();
}

void MPU6050_Calibrate(int16_t* gyroX_offset, int16_t* gyroY_offset, int16_t* gyroZ_offset)
{
	const int num_samples = 1000;
	int32_t gyroX_sum = 0, gyroY_sum = 0, gyroZ_sum = 0;

	for (int i = 0; i < num_samples; i++)
	{
		int16_t AccelX, AccelY, AccelZ, GyroX, GyroY, GyroZ;
		MPU6050_Read_RawData(&AccelX, &AccelY, &AccelZ, &GyroX, &GyroY, &GyroZ);

		gyroX_sum += GyroX;
		gyroY_sum += GyroY;
		gyroZ_sum += GyroZ;

		_delay_ms(1);
	}

	*gyroX_offset = gyroX_sum / num_samples;
	*gyroY_offset = gyroY_sum / num_samples;
	*gyroZ_offset = gyroZ_sum / num_samples;
}

void MPU6050_Read_RawData(int16_t* AccelX, int16_t* AccelY, int16_t* AccelZ, int16_t* GyroX, int16_t* GyroY, int16_t* GyroZ)
{
	I2C_Start_Wait(MPU6050_ADDR << 1);
	I2C_Write(MPU6050_RA_ACCEL_XOUT_H);
	I2C_Stop();

	I2C_Start(MPU6050_ADDR << 1 | 0x01);
	*AccelX = ((int16_t)I2C_Read_Ack() << 8) | I2C_Read_Ack();
	*AccelY = ((int16_t)I2C_Read_Ack() << 8) | I2C_Read_Ack();
	*AccelZ = ((int16_t)I2C_Read_Ack() << 8) | I2C_Read_Ack();
	*GyroX = ((int16_t)I2C_Read_Ack() << 8) | I2C_Read_Ack();
	*GyroY = ((int16_t)I2C_Read_Ack() << 8) | I2C_Read_Ack();
	*GyroZ = ((int16_t)I2C_Read_Ack() << 8) | I2C_Read_Nack();
	I2C_Stop();
}

/* PWM and motor control functions */
void PWM_Init()
{
	DDRD |= (1 << IN1) | (1 << IN2) | (1 << ENB);
	DDRB |= (1 << IN3) | (1 << IN4) | (1 << ENA);
	
	TCCR0A |= (1 << COM0A1) | (1 << WGM00) | (1 << WGM01);
	TCCR0B |= (1 << CS01);
	TCCR2A |= (1 << COM2B1) | (1 << WGM20) | (1 << WGM21);
	TCCR2B |= (1 << CS21);
}

void Motor_SetSpeed(uint8_t left_speed, uint8_t right_speed)
{
	OCR0A = left_speed; // ENB
	OCR2B = right_speed; // ENA
}

void Motor_SetDirection(bool forward)
{
	if(forward)
	{
		PORTD |= (1 << IN1);
		PORTD &= ~(1 << IN2);
		PORTB |= (1 << IN3);
		PORTB &= ~(1 << IN4);
	}
	else
	{
		PORTD &= ~(1 << IN1);
		PORTD |= (1 << IN2);
		PORTB &= ~(1 << IN3);
		PORTB |= (1 << IN4);
	}
}

void Motor_Stop()
{
	PORTD &= ~((1 << IN1) | (1 << IN2));
	PORTB &= ~((1 << IN3) | (1 << IN4));
}

/* PID controller function */
float PID_Controller(float setpoint, float measured, float* previous_error, float* integral)
{
	float error = setpoint - measured;
	*integral += error * 0.01;
	float derivative = (error - *previous_error) / 0.01;
	float output = Kp * error + Ki * *integral + Kd * derivative;
	*previous_error = error;                             
	return output;
}