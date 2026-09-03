#ifndef INC_ENCODER_HPP_
#define INC_ENCODER_HPP_

#include "stm32f4xx_hal.h"

class Encoder {
public:
	Encoder(TIM_HandleTypeDef *htim, uint32_t counterPeriod);

	// Запуск аппаратного квадратурного счётчика и сброс позиции.
	void init();

	// Текущее значение аппаратного счётчика.
	int32_t count() const;

	// Число отсчётов с предыдущего вызова.
	int32_t readDelta();

	// Сброс счётчика и внутренней точки отсчёта.
	void reset();

private:
	TIM_HandleTypeDef *m_htim;
	uint32_t m_counterPeriod;
	uint32_t m_previousCount;
};

#endif /* INC_ENCODER_HPP */
