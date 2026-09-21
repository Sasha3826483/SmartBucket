#include "mlx90614.hpp"

MLX90614::MLX90614(I2C_HandleTypeDef* i2c, uint8_t address)
    // Конструктор базового класса I2CDevice с указанием I2C и адреса устройства
	: I2CDevice{i2c, address} {
}

HAL_StatusTypeDef MLX90614::readObjectTemperature(float& temperatureCelsius,
		uint32_t timeoutMs) {
	return readTemperature(ObjectTemperatureRegister, temperatureCelsius,
			timeoutMs);
}

HAL_StatusTypeDef MLX90614::readAmbientTemperature(float& temperatureCelsius,
		uint32_t timeoutMs) {
	return readTemperature(AmbientTemperatureRegister, temperatureCelsius,
			timeoutMs);
}

bool MLX90614::isConnected(uint32_t timeoutMs) const {
    // Проверяем готовность устройства к обмену данными по I2C c
    // помощью базового метода isReady() из класса I2CDevice
	return isReady(timeoutMs);
}

HAL_StatusTypeDef MLX90614::readTemperature(uint8_t registerAddress,
		float& temperatureCelsius, uint32_t timeoutMs) const {
    // Буфер для хранения данных, считанных из регистра устройства
	uint8_t buffer[3]{};
    // Читаем данные из указанного регистра устройства с помощью базового метода 
    // readRegister() из класса I2CDevice
	HAL_StatusTypeDef status = readRegister(registerAddress, buffer,
			sizeof(buffer), timeoutMs);
	if (status != HAL_OK)
		return status;

	uint8_t pecData[5] {
		static_cast<uint8_t>(getAddress() << 1U),
		registerAddress,
		static_cast<uint8_t>((getAddress() << 1U) | 1U),
		buffer[0],
		buffer[1]
	};

	if (calculatePec(pecData, sizeof(pecData)) != buffer[2])
		return HAL_ERROR;

	uint16_t rawValue = static_cast<uint16_t>(buffer[0]) |
			(static_cast<uint16_t>(buffer[1]) << 8U);

    // Преобразуем сырое значение температуры в градусы Цельсия по формуле:
    // Температура (°C) = (Сырое значение * 0.02) - 273.15. Здесь 0.02 - это 
    // коэффициент масштабирования, а 273.15 - это смещение для перевода из 
    // Кельвинов в Цельсии.
	temperatureCelsius = (static_cast<float>(rawValue) * 0.02f) - 273.15f;

	return HAL_OK;
}

uint8_t MLX90614::calculatePec(const uint8_t* data, uint8_t size) {
	uint8_t remainder = 0U;
	for (uint8_t byteIndex = 0; byteIndex < size; ++byteIndex) {
		remainder ^= data[byteIndex];
		for (uint8_t bitIndex = 0; bitIndex < 8U; ++bitIndex) {
			if ((remainder & 0x80U) != 0U)
				remainder = static_cast<uint8_t>((remainder << 1U) ^ 0x07U);
			else
				remainder = static_cast<uint8_t>(remainder << 1U);
		}
	}
	return remainder;
}
