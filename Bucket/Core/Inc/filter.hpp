#ifndef FILTER_HPP
#define FILTER_HPP

#include <stdint.h>

// Интерфейс фильтра
class IFilter {
public:
    // Виртуальный деструктор для корректного удаления объектов через указатель на базовый класс
	virtual ~IFilter() = default;

    // Чисто виртуальные методы, которые должны быть реализованы в производных классах
    // Метод для обновления фильтра с новым входным значением
	virtual float update(float value) = 0;
    // Метод для сброса состояния фильтра с возможностью установки начального значения
	virtual void reset(float value = 0.0f) = 0;
    // Метод для получения текущего значения фильтра
	virtual float getValue() const = 0;
};

// Медианный фильтр (Median Filter)
template <uint8_t WindowSize>
class MedianFilter final : public IFilter {
public:
	float update(float input) override {
		if (!m_initialized) {
			for (uint8_t i = 0; i < WindowSize; ++i)
				m_samples[i] = input;
			m_initialized = true;
			m_value = input;
			return m_value;
		}

		m_samples[m_index] = input;
		m_index = static_cast<uint8_t>((m_index + 1U) % WindowSize);

		float sortedSamples[WindowSize];
		for (uint8_t i = 0; i < WindowSize; ++i)
			sortedSamples[i] = m_samples[i];

		for (uint8_t i = 1; i < WindowSize; ++i) {
			float current = sortedSamples[i];
			uint8_t j = i;
			while (j > 0 && sortedSamples[j - 1] > current) {
				sortedSamples[j] = sortedSamples[j - 1];
				--j;
			}
			sortedSamples[j] = current;
		}

		m_value = sortedSamples[WindowSize / 2U];
		return m_value;
	}

	void reset(float input = 0.0f) override {
		m_index = 0;
		m_initialized = false;
		m_value = input;
	}

	float getValue() const override {
		return m_value;
	}

private:
	static_assert(WindowSize > 0, "MedianFilter window size must be greater than zero");

	float m_samples[WindowSize]{};
	float m_value{0.0f};
	uint8_t m_index{0};
	bool m_initialized{false};
};

// Фильтр среднего значения (Moving Average Filter)
template <uint8_t WindowSize>
class MovingAverageFilter final : public IFilter {
public:
	float update(float input) override {
		if (!m_initialized) {
			for (uint8_t i = 0; i < WindowSize; ++i)
				m_samples[i] = input;
			m_sum = input * WindowSize;
			m_initialized = true;
		} else {
			m_sum -= m_samples[m_index];
			m_samples[m_index] = input;
			m_sum += input;
		}

		m_index = static_cast<uint8_t>((m_index + 1U) % WindowSize);
		m_value = m_sum / WindowSize;
		return m_value;
	}

	void reset(float input = 0.0f) override {
		m_index = 0;
		m_sum = 0.0f;
		m_value = input;
		m_initialized = false;
	}

	float getValue() const override {
		return m_value;
	}

private:
	static_assert(WindowSize > 0, "MovingAverageFilter window size must be greater than zero");

	float m_samples[WindowSize]{};
	float m_sum{0.0f};
	float m_value{0.0f};
	uint8_t m_index{0};
	bool m_initialized{false};
};

// Фильтр экспоненциального сглаживания (Exponential Filter)
class ExponentialFilter final : public IFilter {
public:
	explicit ExponentialFilter(float alpha)
		: m_alpha{alpha} {
	}

	float update(float input) override {
		if (!m_initialized) {
			m_value = input;
			m_initialized = true;
		} else {
			m_value += m_alpha * (input - m_value);
		}

		return m_value;
	}

	void reset(float input = 0.0f) override {
		m_value = input;
		m_initialized = false;
	}

	float getValue() const override {
		return m_value;
	}

private:
	float m_alpha;
	float m_value{0.0f};
	bool m_initialized{false};
};

#endif // FILTER_HPP