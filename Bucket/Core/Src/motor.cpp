/*
 * motor.cpp
 *
 *  Created on: Aug 5, 2026
 *      Author: Саша
 */

#include "motor.hpp"

Motor::Motor(TIM_HandleTypeDef *htim, uint32_t channel, GPIO_TypeDef *dirPort1,
		uint16_t dirPin1, GPIO_TypeDef *dirPort2, uint16_t dirPin2) :
		m_pwm{htim, channel}, m_dirPort1{dirPort1}, m_dirPin1{dirPin1},
		m_dirPort2{dirPort2}, m_dirPin2{dirPin2} {
}

HAL_StatusTypeDef Motor::init() {
	const HAL_StatusTypeDef status{m_pwm.start()};
	if (status != HAL_OK)
		return status;

	stop();
	return HAL_OK;
}

void Motor::setSpeed(uint8_t percent) {
	if (percent > 100)
		percent = 100;
	m_pwm.setCompare(m_pwm.periodTicks() * percent / 100);
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
