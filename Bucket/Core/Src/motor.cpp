/*
 * motor.cpp
 *
 *  Created on: Aug 5, 2026
 *      Author: Саша
 */

#include "Motor.hpp"

Motor::Motor(TIM_HandleTypeDef *htim, uint32_t channel, GPIO_TypeDef *dirPort1,
		uint16_t dirPin1, GPIO_TypeDef *dirPort2, uint16_t dirPin2) :
		m_htim(htim), m_channel(channel), m_dirPort1(dirPort1), m_dirPin1(
				dirPin1), m_dirPort2(dirPort2), m_dirPin2(dirPin2), m_speedPercent(
				0) {
}

void Motor::init() {
	HAL_TIM_PWM_Start(m_htim, m_channel);
	stop();
}

void Motor::setSpeed(uint8_t percent) {
	if (percent > 100)
		percent = 100;
	m_speedPercent = percent;

	// Вычисление CCR на основе текущего значения ARR таймера
	uint32_t pulse = (__HAL_TIM_GET_AUTORELOAD(m_htim) + 1) * percent / 100;
	__HAL_TIM_SET_COMPARE(m_htim, m_channel, pulse);
}

void Motor::setDirection(MotorDirection dir) {
	switch (dir) {
	case MotorDirection::FORWARD:
		HAL_GPIO_WritePin(m_dirPort1, m_dirPin1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(m_dirPort2, m_dirPin2, GPIO_PIN_RESET);
		break;
	case MotorDirection::BACKWARD:
		HAL_GPIO_WritePin(m_dirPort1, m_dirPin1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(m_dirPort2, m_dirPin2, GPIO_PIN_SET);
		break;
	case MotorDirection::STOP:
		HAL_GPIO_WritePin(m_dirPort1, m_dirPin1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(m_dirPort2, m_dirPin2, GPIO_PIN_SET);
		break;
	}
}

void Motor::stop() {
	setSpeed(0);
	setDirection(MotorDirection::STOP);
}
