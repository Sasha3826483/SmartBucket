#ifndef I2C_DEVICE_HPP
#define I2C_DEVICE_HPP

#include "stm32f4xx_hal.h"

class I2CDevice {
public:
	static constexpr uint32_t DefaultTimeoutMs = 10U;

    // Конструктор класса I2CDevice, принимающий указатель на структуру I2C_HandleTypeDef
    // и адрес устройства
	I2CDevice(I2C_HandleTypeDef* i2c, uint8_t address);
    
    // Виртуальный деструктор для корректного удаления объектов через указатель
	virtual ~I2CDevice() = default;

    // Метод для проверки готовности устройства к обмену данными по I2C
	bool isReady(uint32_t timeoutMs = DefaultTimeoutMs) const;

    // Метод для получения адреса устройства
	uint8_t getAddress() const {
		return m_address;
	}

protected:
    // Метод для чтения данных из регистра устройства по I2C
	HAL_StatusTypeDef readRegister(uint8_t registerAddress,
			uint8_t* data, uint16_t size,
			uint32_t timeoutMs = DefaultTimeoutMs) const;

    // Метод для записи данных в регистр устройства по I2C
	HAL_StatusTypeDef writeRegister(uint8_t registerAddress,
			const uint8_t* data, uint16_t size,
			uint32_t timeoutMs = DefaultTimeoutMs) const;

    // Метод для чтения данных из устройства по I2C
	HAL_StatusTypeDef read(uint8_t* data, uint16_t size,
			uint32_t timeoutMs = DefaultTimeoutMs) const;

    // Метод для записи данных в устройство по I2C
	HAL_StatusTypeDef write(const uint8_t* data, uint16_t size,
			uint32_t timeoutMs = DefaultTimeoutMs) const;

    // Метод для получения указателя на структуру I2C_HandleTypeDef
	I2C_HandleTypeDef* getI2C() const {
		return m_i2c;
	}

private:
	I2C_HandleTypeDef* m_i2c;
	uint8_t m_address;
};

#endif // I2C_DEVICE_HPP