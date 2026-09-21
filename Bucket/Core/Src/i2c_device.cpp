#include "i2c_device.hpp"

I2CDevice::I2CDevice(I2C_HandleTypeDef* i2c, uint8_t address)
	: m_i2c{i2c}, m_address{address} {
}

bool I2CDevice::isReady(uint32_t timeoutMs) const {
	return HAL_I2C_IsDeviceReady(m_i2c,
			static_cast<uint16_t>(m_address << 1U),
			2U, timeoutMs) == HAL_OK;
}

HAL_StatusTypeDef I2CDevice::readRegister(uint8_t registerAddress,
		uint8_t* data, uint16_t size, uint32_t timeoutMs) const {
	return HAL_I2C_Mem_Read(
		m_i2c,
		static_cast<uint16_t>(m_address << 1U),
		registerAddress,
		I2C_MEMADD_SIZE_8BIT,
		data,
		size,
		timeoutMs);
}

HAL_StatusTypeDef I2CDevice::writeRegister(uint8_t registerAddress,
		const uint8_t* data, uint16_t size, uint32_t timeoutMs) const {
	return HAL_I2C_Mem_Write(
		m_i2c,
		static_cast<uint16_t>(m_address << 1U),
		registerAddress,
		I2C_MEMADD_SIZE_8BIT,
		const_cast<uint8_t*>(data),
		size,
		timeoutMs);
}

HAL_StatusTypeDef I2CDevice::read(uint8_t* data, uint16_t size,
		uint32_t timeoutMs) const {
	return HAL_I2C_Master_Receive(
		m_i2c,
		static_cast<uint16_t>(m_address << 1U),
		data,
		size,
		timeoutMs);
}

HAL_StatusTypeDef I2CDevice::write(const uint8_t* data, uint16_t size,
		uint32_t timeoutMs) const {
	return HAL_I2C_Master_Transmit(
		m_i2c,
		static_cast<uint16_t>(m_address << 1U),
		const_cast<uint8_t*>(data),
		size,
		timeoutMs);
}