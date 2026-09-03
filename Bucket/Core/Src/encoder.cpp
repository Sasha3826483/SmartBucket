#include "encoder.hpp"

Encoder::Encoder(TIM_HandleTypeDef *htim, uint32_t counterPeriod) :
		m_htim{htim}, m_counterPeriod{counterPeriod}, m_previousCount{0} {
}

void Encoder::init() {
	HAL_TIM_Encoder_Start(m_htim, TIM_CHANNEL_ALL);
	reset();
}

int32_t Encoder::count() const {
	return static_cast<int32_t>(__HAL_TIM_GET_COUNTER(m_htim));
}

int32_t Encoder::readDelta() {
	uint32_t currentCount = __HAL_TIM_GET_COUNTER(m_htim);
	uint64_t counterRange = static_cast<uint64_t>(m_counterPeriod) + 1ULL;
	int64_t delta = static_cast<int64_t>(currentCount)
			- static_cast<int64_t>(m_previousCount);

	if (delta > static_cast<int64_t>(counterRange / 2ULL))
		delta -= static_cast<int64_t>(counterRange);
	else if (delta < -static_cast<int64_t>(counterRange / 2ULL))
		delta += static_cast<int64_t>(counterRange);

	m_previousCount = currentCount;
	return static_cast<int32_t>(delta);
}

void Encoder::reset() {
	__HAL_TIM_SET_COUNTER(m_htim, 0);
	m_previousCount = 0;
}
