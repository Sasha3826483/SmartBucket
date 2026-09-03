/*
 * motor.hpp
 *
 *  Created on: Aug 5, 2026
 *      Author: Саша
 */

#ifndef INC_MOTOR_HPP_
#define INC_MOTOR_HPP_

#include "stm32f4xx_hal.h"

enum class MotorDirection {
	FORWARD, BACKWARD, STOP
};

class Motor {
public:
	// Конструктор принимает аппаратные ресурсы
	Motor(TIM_HandleTypeDef *htim, uint32_t channel, GPIO_TypeDef *dirPort1,
			uint16_t dirPin1, GPIO_TypeDef *dirPort2, uint16_t dirPin2);

	// Инициализация (запуск ШИМ)
	void init();

	// Управление скоростью: dutyCycle от 0 до 100%
	void setSpeed(uint8_t percent);

	// Управление направлением
	void setDirection(MotorDirection dir);

	// Быстрый останов
	void stop();

private:
	TIM_HandleTypeDef *m_htim;
	uint32_t m_channel;

	GPIO_TypeDef *m_dirPort1;
	uint16_t m_dirPin1;

	GPIO_TypeDef *m_dirPort2;
	uint16_t m_dirPin2;

	uint8_t m_speedPercent;
};

#endif /* INC_MOTOR_HPP_ */
