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
	// Получаем текущее значение счетчика энкодера
	uint32_t currentCount = __HAL_TIM_GET_COUNTER(m_htim);

	// Вычисляем диапазон счетчика, учитывая, что он может быть 16-битным или 32-битным
	uint64_t counterRange = static_cast<uint64_t>(m_counterPeriod) + 1ULL;

	// Вычисляем дельту между текущим значением и предыдущим значением счетчика
	int64_t delta = static_cast<int64_t>(currentCount)
			- static_cast<int64_t>(m_previousCount);

	// Обрабатываем переполнение счетчика
	// Если дельта больше половины диапазона счетчика, значит произошло переполнение в положительном направлении
	if (delta > static_cast<int64_t>(counterRange / 2ULL))
		delta -= static_cast<int64_t>(counterRange);
	// Если дельта меньше отрицательной половины диапазона счетчика, значит произошло переполнение в отрицательном направлении
	else if (delta < -static_cast<int64_t>(counterRange / 2ULL))
		delta += static_cast<int64_t>(counterRange);

	m_previousCount = currentCount;
	return static_cast<int32_t>(delta);
}

void Encoder::reset() {
	__HAL_TIM_SET_COUNTER(m_htim, 0);
	m_previousCount = 0;
}
