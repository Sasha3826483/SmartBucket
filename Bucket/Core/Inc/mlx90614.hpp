#ifndef MLX90614_HPP
#define MLX90614_HPP

#include "i2c_device.hpp"

// Класс для работы с инфракрасным термометром MLX90614 через интерфейс I2C
class MLX90614 final : public I2CDevice {
public:
	static constexpr uint8_t DefaultAddress = 0x5AU;

    // Конструктор класса MLX90614, принимающий указатель на структуру 
    // I2C_HandleTypeDef и адрес устройства (по умолчанию 0x5A)
	MLX90614(I2C_HandleTypeDef* i2c, uint8_t address = DefaultAddress);

    // Метод для чтения температуры объекта в градусах Цельсия
	HAL_StatusTypeDef readObjectTemperature(float& temperatureCelsius,
			uint32_t timeoutMs = 10U);

    // Метод для чтения температуры окружающей среды в градусах Цельсия
	HAL_StatusTypeDef readAmbientTemperature(float& temperatureCelsius,
			uint32_t timeoutMs = 10U);

    // Метод для проверки подключения устройства к шине I2C
	bool isConnected(uint32_t timeoutMs = DefaultTimeoutMs) const;

private:
    // Адреса регистров для чтения температуры
	static constexpr uint8_t AmbientTemperatureRegister = 0x06U; // Адрес регистра температуры окружающей среды
	static constexpr uint8_t ObjectTemperatureRegister = 0x07U; // Адрес регистра температуры объекта
        
    // Внутренний метод для чтения температуры из указанного регистра
	HAL_StatusTypeDef readTemperature(uint8_t registerAddress,
			float& temperatureCelsius, uint32_t timeoutMs) const;
    
    // Метод для вычисления контрольной суммы PEC (Packet Error Code) 
    // для проверки целостности данных
	static uint8_t calculatePec(const uint8_t* data, uint8_t size);
};

#endif // MLX90614_HPP