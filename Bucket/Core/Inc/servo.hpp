#ifndef INC_SERVO_HPP_
#define INC_SERVO_HPP_

#include "pwm_output.hpp"

class Servo {
public:
	Servo(PwmOutput pwm, uint32_t minPulseTicks, uint32_t maxPulseTicks) :
			m_pwm{pwm}, m_minPulseTicks{minPulseTicks}, m_maxPulseTicks{
					maxPulseTicks} {
	}

	HAL_StatusTypeDef init(uint32_t initialPulseTicks) {
		const HAL_StatusTypeDef status{m_pwm.start()};
		if (status != HAL_OK)
			return status;

		setPulseTicks(initialPulseTicks);
		return HAL_OK;
	}

	void setAngle(uint8_t degrees) {
		if (degrees > 180)
			degrees = 180;

		const uint32_t pulseTicks{m_minPulseTicks
				+ (static_cast<int32_t>(m_maxPulseTicks) - static_cast<int32_t>(m_minPulseTicks)) * degrees / 180};
		m_pwm.setCompare(pulseTicks);
	}

	void setPulseTicks(uint32_t pulseTicks) {
		
		if (m_maxPulseTicks > m_minPulseTicks) {
			if (pulseTicks < m_minPulseTicks)
				pulseTicks = m_minPulseTicks;
			if (pulseTicks > m_maxPulseTicks)
				pulseTicks = m_maxPulseTicks;
		} else {
			if (pulseTicks > m_minPulseTicks)
				pulseTicks = m_minPulseTicks;
			if (pulseTicks < m_maxPulseTicks)
				pulseTicks = m_maxPulseTicks;
		}

		m_pwm.setCompare(pulseTicks);
	}

private:
	PwmOutput m_pwm;
	uint32_t m_minPulseTicks;
	uint32_t m_maxPulseTicks;
};

#endif /* INC_SERVO_HPP_ */
