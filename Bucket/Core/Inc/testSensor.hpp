#ifndef testI2C_HPP
#define testI2C_HPP

#include "i2c_device.hpp"

class testI2C final : public I2CDevice {
public:
	static constexpr uint8_t DefaultAddress = 0x5U;

	testI2C(I2C_HandleTypeDef* i2c, uint8_t address = DefaultAddress);

	HAL_StatusTypeDef readTest(float& value,
			uint32_t timeoutMs = 10U);

	bool isConnected(uint32_t timeoutMs = DefaultTimeoutMs) const;
		
private:
	static constexpr uint8_t testValueRegister = 0x00U;
        
	HAL_StatusTypeDef readTestValue(uint8_t registerAddress,
			float& temperatureCelsius, uint32_t timeoutMs) const;
};

#endif // testI2C_HPP