#ifndef INC_PWM_OUTPUT_HPP_
#define INC_PWM_OUTPUT_HPP_

#include "stm32f4xx_hal.h"

class PwmOutput {
public:
	PwmOutput(TIM_HandleTypeDef *htim, uint32_t channel) :
			m_htim{htim}, m_channel{channel} {
	}

	HAL_StatusTypeDef start() {
		return HAL_TIM_PWM_Start(m_htim, m_channel);
	}

	uint32_t periodTicks() const {
		return __HAL_TIM_GET_AUTORELOAD(m_htim) + 1;
	}

	void setCompare(uint32_t ticks) {
		__HAL_TIM_SET_COMPARE(m_htim, m_channel, ticks);
	}

private:
	TIM_HandleTypeDef *m_htim;
	uint32_t m_channel;
};

#endif /* INC_PWM_OUTPUT_HPP_ */
