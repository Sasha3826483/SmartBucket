#include <testSensor.hpp>

testI2C::testI2C(I2C_HandleTypeDef* i2c, uint8_t address)
    // Конструктор базового класса I2CDevice с указанием I2C и адреса устройства
	: I2CDevice{i2c, address} {
}

HAL_StatusTypeDef testI2C::readTest(float& value,
		uint32_t timeoutMs) {
	return readTestValue(testValueRegister, value,
			timeoutMs);
}

bool testI2C::isConnected(uint32_t timeoutMs) const {
	return isReady(timeoutMs);
}

HAL_StatusTypeDef testI2C::readTestValue(uint8_t registerAddress,
		float& value, uint32_t timeoutMs) const {
	
    // Буфер для хранения данных, считанных из регистра устройства
	uint8_t buffer[1]{};
    // Читаем данные из указанного регистра устройства с помощью базового метода 
    // readRegister() из класса I2CDevice
	HAL_StatusTypeDef status = readRegister(registerAddress, buffer,
			sizeof(buffer), timeoutMs);
	if (status != HAL_OK)
		return status;

	uint16_t rawValue = static_cast<uint16_t>(buffer[0]);

	value = rawValue;

	return HAL_OK;
}
