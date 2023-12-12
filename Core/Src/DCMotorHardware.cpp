/*
 * DCMotorHardware.cpp
 *
 *  Created on: May 7, 2020
 *      Author: victor
 */

#include "DCMotorHardware.h"
extern "C" {
	#include "main.h" // Include pin definitions
}
DCMotorHardware::DCMotorHardware(TIM_TypeDef* a_encoder_right_timer,
		TIM_TypeDef* a_encoder_left_timer,
		TIM_HandleTypeDef* a_motor_right_timer,
		const int32_t a_motor_right_timer_channel,
		TIM_HandleTypeDef* a_motor_left_timer,
		const int32_t a_motor_left_timer_channel) {
	dir_right_gpio_bank = DIR_A_GPIO_Port;
	dir_right_gpio = DIR_A_Pin;
	dir_left_gpio_bank = DIR_B_GPIO_Port;
	dir_left_gpio = DIR_B_Pin;
	encoder_right_timer = a_encoder_right_timer;
	encoder_left_timer = a_encoder_left_timer;
	motor_right_timer = a_motor_right_timer;
	motor_right_timer_channel = a_motor_right_timer_channel;
	motor_left_timer = a_motor_left_timer;
	motor_left_timer_channel = a_motor_left_timer_channel;

	HAL_GPIO_WritePin(BRAKE_GPIO_Port, BRAKE_Pin, GPIO_PIN_RESET);//BRAKE
}
DCMotorHardware::DCMotorHardware() {}
DCMotorHardware::~DCMotorHardware() {
	HAL_GPIO_WritePin(BRAKE_GPIO_Port, BRAKE_Pin, GPIO_PIN_RESET);//BRAKE
}

int16_t DCMotorHardware::getTicks(const uint32_t encoderId) {
	if (encoderId == M_L) {
		return encoder_left_timer->CNT;
	}
	return encoder_right_timer->CNT;
}

uint32_t DCMotorHardware::getMilliSecondsElapsed() {
	return HAL_GetTick();
}

void DCMotorHardware::setPWM(const int32_t pwm_left, const int32_t pwm_right) {
	// Write dir according to pwm sign
	if (pwm_left > 0) {
		HAL_GPIO_WritePin(dir_left_gpio_bank, dir_left_gpio, GPIO_PIN_SET);
	}
	else {
		HAL_GPIO_WritePin(dir_left_gpio_bank, dir_left_gpio, GPIO_PIN_RESET);
	}

	if (pwm_right > 0) {
		HAL_GPIO_WritePin(dir_right_gpio_bank, dir_right_gpio, GPIO_PIN_SET);
	}
	else {
		HAL_GPIO_WritePin(dir_right_gpio_bank, dir_right_gpio, GPIO_PIN_RESET);
	}

	// Limit to pwm boundaries and write PWM
	int32_t command = 0;
	command = MAX(pwm_left, -pwm_left);
	command = MIN(command, DUTYMAX);
	__HAL_TIM_SET_COMPARE(motor_left_timer, motor_left_timer_channel, command);

	command = MAX(pwm_right, -pwm_right);
	command = MIN(command, DUTYMAX);
	__HAL_TIM_SET_COMPARE(motor_right_timer, motor_right_timer_channel, command);

	if(pwm_left == 0 && pwm_right == 0) {
		HAL_GPIO_WritePin(BRAKE_GPIO_Port, BRAKE_Pin, GPIO_PIN_RESET);//BRAKE
	}
	else {
		HAL_GPIO_WritePin(BRAKE_GPIO_Port, BRAKE_Pin, GPIO_PIN_SET);//un-BRAKE
	}
}
