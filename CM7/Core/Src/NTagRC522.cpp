#include "NTagRC522.h"



//write 8 bits to the specified address
uint8_t NTagRC522::WriteRegister(uint8_t reg_addr, uint8_t write_data) {
	uint8_t transmit[2];
	transmit[0] = reg_addr << 1;
	transmit[1] = write_data;

	uint8_t ret;
	ret = HAL_SPI_Transmit(SPIHandle, transmit, 2, HAL_MAX_DELAY);
	//ret = HAL_I2C_Master_Transmit(I2CHandle, i2c_addr, &reg_addr, 1, HAL_MAX_DELAY);
	//if(ret != HAL_OK) panic

	return ret;
}
//read 8 bits from the specified address
uint8_t NTagRC522::ReadRegister(uint8_t reg_addr) {
	//Since my spi is full duplex, I need two bits for send/recv. One is dummy.
	uint8_t received[2];
	uint8_t transmit[2];
	transmit[0] = (reg_addr << 1) | 0x80; //set first bit to denote read
	transmit[1] = 0;

	uint8_t ret;
	ret = HAL_SPI_TransmitReceive(SPIHandle, transmit, received, 2, HAL_MAX_DELAY);
	//if(ret != HAL_OK) panic

	/*uint8_t ret;
	ret = HAL_SPI_Transmit(SPIHandle, &reg_addr, 1, HAL_MAX_DELAY);
	//if(ret != HAL_OK) panic
	uint8_t received[2];
	ret = HAL_SPI_Receive(SPIHandle, received, 2, HAL_MAX_DELAY);
	//if(ret != HAL_OK) panic*/

	return received[1];
}

//end of private functions

void NTagRC522::SetSPIHandle(SPI_HandleTypeDef *spi) {
	SPIHandle = spi;
}
//setup the dumb thing to operate. Run after any resetting.
void NTagRC522::Init() {

}
//reads and returns the version register value
uint8_t NTagRC522::SanityCheck() {
	return this->ReadRegister(this->VersionReg);
}
uint8_t NTagRC522::SanityCheck2() {
	//return this->ReadRegister(0x3C);
	return this->ReadRegister(0x2E);
}

uint8_t NTagRC522::GetWaterLevel() {
	return this->ReadRegister(this->WaterLevelReg);
}
uint8_t NTagRC522::SetWaterLevel(uint8_t set_value) {
	return this->WriteRegister(this->WaterLevelReg, set_value);
}

