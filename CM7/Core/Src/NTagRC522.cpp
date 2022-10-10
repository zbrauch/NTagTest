#include "NTagRC522.h"



//write 8 bits to the specified address
uint8_t NTagRC522::writeRegister(uint8_t i2c_addr, uint8_t reg_addr) {
	uint8_t ret;
	ret = HAL_I2C_Master_Transmit(I2CHandle, i2c_addr, &reg_addr, 1, HAL_MAX_DELAY);
	//if(ret != HAL_OK) panic

	return ret;
}
//read 8 bits from the specified address
uint8_t NTagRC522::readRegister(uint8_t i2c_addr, uint8_t reg_addr) {
	uint8_t read_result, ret;
	ret = HAL_I2C_Master_Transmit(I2CHandle, i2c_addr, &reg_addr, 1, HAL_MAX_DELAY);
	if(ret != HAL_OK) __asm("nop");//panic
	ret = HAL_I2C_Master_Receive(I2CHandle, i2c_addr, &read_result, 1, HAL_MAX_DELAY);
	if(ret != HAL_OK) __asm("nop");//panic
	return read_result;
}

//end of private functions

void NTagRC522::SetI2CHandle(I2C_HandleTypeDef *i2c) {
	I2CHandle = i2c;
}
//setup the dumb thing to operate. Run after any resetting.
void NTagRC522::Init(uint8_t i2c_addr) {

}
//reads and returns the version register value
uint8_t NTagRC522::SanityCheck(uint8_t i2c_addr) {
	return this->readRegister(i2c_addr, this->VersionReg);
}
