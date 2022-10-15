#include "NTagRC522.h"
#include "cmsis_os.h"


//write 8 bits to the specified address
uint8_t NTagRC522::WriteRegister(uint8_t reg_addr, uint8_t write_data) {
	EnableCS();
	uint8_t transmit[2];
	transmit[0] = reg_addr << 1;
	transmit[1] = write_data;

	uint8_t ret;
	ret = HAL_SPI_Transmit(SPIHandle, transmit, 2, HAL_MAX_DELAY);
	//ret = HAL_I2C_Master_Transmit(I2CHandle, i2c_addr, &reg_addr, 1, HAL_MAX_DELAY);
	//if(ret != HAL_OK) panic
	DisableCS();
	return ret;
}
//write multiple bytes to the specified address. Max of 255
uint8_t NTagRC522::WriteRegister(uint8_t reg_addr, uint8_t write_len, uint8_t *write_data) {
	uint8_t transmit[256];
	transmit[0] = reg_addr << 1;
	for(int i = 0; i < write_len; i++)
		transmit[i+1] = write_data[i];

	uint8_t ret;
	EnableCS();
	ret = HAL_SPI_Transmit(SPIHandle, transmit, write_len+1, HAL_MAX_DELAY);
	//ret = HAL_I2C_Master_Transmit(I2CHandle, i2c_addr, &reg_addr, 1, HAL_MAX_DELAY);
	//if(ret != HAL_OK) panic
	DisableCS();
	return ret;
}

/*uint8_t NTagRC522::WriteRegister(uint8_t reg_addr, uint8_t write_len, uint8_t *write_data) {
	for(uint8_t i = 0; i < write_len; i++)
		WriteRegister(reg_addr, write_data[i]);
}*/

//Set bits in mask
uint8_t NTagRC522::SetBitsRegister(uint8_t reg_addr, uint8_t mask) {
	return WriteRegister(reg_addr, ReadRegister(reg_addr) | mask);
}
//Clear bits in mask (1s in mask are cleared)
uint8_t NTagRC522::ClearBitsRegister(uint8_t reg_addr, uint8_t mask) {
	return WriteRegister(reg_addr, ReadRegister(reg_addr) & (~mask));
}
//read 8 bits from the specified address
uint8_t NTagRC522::ReadRegister(uint8_t reg_addr) {
	EnableCS();
	//Since my spi is full duplex, I need two bits for send/recv. One is dummy.
	uint8_t received[2];
	uint8_t transmit[2];
	transmit[0] = (reg_addr << 1) | 0x80; //set first bit to denote read
	transmit[1] = 0;

	uint8_t error;
	error = HAL_SPI_TransmitReceive(SPIHandle, transmit, received, 2, HAL_MAX_DELAY);
	//if(error != HAL_OK) panic
	DisableCS();
	return received[1];
}
//read multiple bytes from the specified address. Max of 255
uint8_t NTagRC522::ReadRegister(uint8_t reg_addr, uint8_t count, uint8_t *values, uint8_t rxAlign) {
	if(count == 0)
		return 0;

	uint8_t received[256];
	uint8_t transmit[256];
	for(int i = 0; i < count; i++) {
		transmit[i] = (reg_addr << 1) | 0x80; //set first bit to denote read
	}
	transmit[count+1] = 0;

	EnableCS();
	uint8_t error;
	error = HAL_SPI_TransmitReceive(SPIHandle, transmit, received, count+1, HAL_MAX_DELAY);
	//if(error != HAL_OK) panic
	DisableCS();


	for(int i = 1; i < count; i++)
		values[i] = received[i+1];

	//rxAlign nonsense
	uint8_t mask = (0xFF << rxAlign) & 0xFF;
	values[0] = (values[0] & ~mask) | (received[1] & mask); //don't mess with masked out bits

	return error;
}

uint8_t NTagRC522::CommunicateWithPICC(uint8_t command, uint8_t waitIRq, uint8_t *sendData, uint8_t sendLen, uint8_t *backData,
			uint8_t *backLen, uint8_t *validBits, uint8_t rxAlign = 0) {
	//prepare for BitFramingReg
	uint8_t txLastBits = validBits ? *validBits : 0;
	uint8_t bitFraming = (rxAlign << 4) + txLastBits;		// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

	WriteRegister(CommandReg, ComIdle);			// Stop any active command.
	WriteRegister(ComIrqReg, 0x7F);					// Clear all seven interrupt request bits
	WriteRegister(FIFOLevelReg, 0x80);				// FlushBuffer = 1, FIFO initialization
	WriteRegister(FIFODataReg, sendLen, sendData);	// Write sendData to the FIFO
	WriteRegister(BitFramingReg, bitFraming);		// Bit adjustments
	WriteRegister(CommandReg, command);				// Execute the command
	if (command == ComTransceive) {
		SetBitsRegister(BitFramingReg, 0x80);	// StartSend=1, transmission of data starts
	}


	//wait for the command to be completed (timer starts -> IRq bit set)
	// ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
	uint32_t deadline = osKernelGetTickCount() + 36; //deadline in ms
	while(!(ReadRegister(ComIrqReg) & waitIRq)) {
		osDelay(1);
		deadline--;
		if(osKernelGetTickCount() > deadline)
			//break;
			return STATUS_TIMEOUT;
	}

	//check for errors
	uint8_t error = ReadRegister(ErrorReg) & 0x13;
	if(error)
		return STATUS_ERROR;

	uint8_t _validBits = 0;

	//send back data if desired
	if(backData && backLen) {
		uint8_t n = ReadRegister(FIFOLevelReg); //get number of bytes
		if(n > *backLen) //error if data is larger than back buffer
			return STATUS_NO_ROOM;

		ReadRegister(FIFODataReg, n, backData, rxAlign);
		_validBits = ReadRegister(ControlReg) & 0x07;
		if(validBits)
			*validBits = _validBits;
	}

	//check if collision
	if(error & 0x08)
		return STATUS_COLLISION;


	//CRC could be performed here.

	return STATUS_OK;
}


uint8_t NTagRC522::TransceiveData(uint8_t *sendData, uint8_t sendLen, uint8_t *backData, uint8_t *backLen,
			uint8_t *validBits, uint8_t rxAlign = 0) {
	uint8_t waitIRq = 0x30; //RxIRq and IdleIRq
	return CommunicateWithPICC(ComTransceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign);
}

uint8_t NTagRC522::CalculateCRC(uint8_t *data, uint8_t length, uint8_t *result) {
	WriteRegister(CommandReg, ComIdle);		// Stop any active command.
	WriteRegister(DivIrqReg, 0x04);				// Clear the CRCIRq interrupt request bit
	WriteRegister(FIFOLevelReg, 0x80);			// FlushBuffer = 1, FIFO initialization
	WriteRegister(FIFODataReg, length, data);	// Write data to the FIFO
	WriteRegister(CommandReg, ComCalcCRC);		// Start the calculation

	// Wait for the CRC calculation to complete. Check for the register to
	// indicate that the CRC calculation is complete in a loop. If the
	// calculation is not indicated as complete in timeout, then time out
	// the operation.
	uint8_t timeout = osKernelGetTickCount() + 15; //might need to be adjusted
	uint8_t n = 0xFF;
	do {
		n = ReadRegister(DivIrqReg);
		if(n & 0x04) {	// CRCIRq bit set - calculation done
			WriteRegister(CommandReg, ComIdle); // Stop calculating CRC for new content in the FIFO.
			// Transfer the result from the registers to the result buffer
			result[0] = ReadRegister(CRCResultRegL);
			result[1] = ReadRegister(CRCResultRegH);
			return STATUS_OK;
		}
		timeout--;

	} while(timeout > osKernelGetTickCount());

	return STATUS_TIMEOUT;
}


//end of private functions


NTagRC522::NTagRC522(SPI_HandleTypeDef *spi, void (*EnableCSFunc)(void), void (*DisableCSFunc)(void)) {
	this->SetSPIHandle(spi);
	this->EnableCS = EnableCSFunc;
	this->DisableCS = DisableCSFunc;
}
void NTagRC522::SetSPIHandle(SPI_HandleTypeDef *spi) {
	SPIHandle = spi;
}
//setup the dumb thing to operate. Run after any resetting.
void NTagRC522::Init() {
	SoftReset();

	//Reset baud rates
	WriteRegister(TxModeReg, 0x00);
	WriteRegister(RxModeReg, 0x00);
	//Reset ModWidthReg
	WriteRegister(ModWidthReg, 0x26);

	//Set a timeout
	//f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
	//TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.
	WriteRegister(TModeReg, 0x80);			// TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
	WriteRegister(TPrescalerReg, 0xA9);		// TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25μs.
	WriteRegister(TReloadRegH, 0x03);		// Reload timer with 0x3E8 = 1000, ie ~8ms before timeout.
	WriteRegister(TReloadRegL, 0xE8);

	WriteRegister(TxASKReg, 0x40);		// Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
	WriteRegister(ModeReg, 0x3D);		// Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
	EnableAntenna();					// Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)
}
//Enable output signal on Tx1 and Tx2
uint8_t NTagRC522::EnableAntenna() {
	return SetBitsRegister(TxControlReg, 0x03);
}
//Disable output signal on Tx1 and Tx2
uint8_t NTagRC522::DisableAntenna() {
	return ClearBitsRegister(TxControlReg, 0x03);
}
//set antenna gain
uint8_t NTagRC522::SetAntennaGain(uint8_t gain) {
	ClearBitsRegister(RFCfgReg, (0x07<<4));
	return SetBitsRegister(RFCfgReg, gain & (0x07<<4));
}

uint8_t NTagRC522::WakeUpA(uint8_t *AToReq, uint8_t *bufferSize) {
	uint8_t validBits, error;
	//set to clear bits after a collision
	ClearBitsRegister(CollReg, 0x80);

	validBits = 7;

	uint8_t command = PICC_CMD_WUPA;
	//uint8_t command = PICC_CMD_REQA;
	error = TransceiveData(&command, 1, AToReq, bufferSize, &validBits);
	//if(error)
		//__asm__("bkpt");

	return error;
}

uint8_t NTagRC522::HaltA() {
	uint8_t result;
	uint8_t buffer[4];

	// Build command buffer
	buffer[0] = PICC_CMD_HLTA;
	buffer[1] = 0;
	// Calculate CRC_A
	result = CalculateCRC(buffer, 2, &buffer[2]);
	if (result != STATUS_OK) {
		return result;
	}

	// Send the command.
	// The standard says:
	//		If the PICC responds with any modulation during a period of 1 ms after the end of the frame containing the
	//		HLTA command, this response shall be interpreted as 'not acknowledge'.
	// We interpret that this way: Only STATUS_TIMEOUT is a success.
	result = TransceiveData(buffer, sizeof(buffer), nullptr, nullptr, nullptr);
	if (result == STATUS_TIMEOUT) {
		return STATUS_OK;
	}
	if (result == STATUS_OK) { // That is ironically NOT ok in this case ;-)
		return STATUS_ERROR;
	}
	return result;
}

uint8_t NTagRC522::SelectPICC(Uid *uid, uint8_t validBits) {
	bool uidComplete, selectDone, useCascadeTag;
	uint8_t cascadeLevel = 1;
	uint8_t result;
	uint8_t count, checkBit, index;
	uint8_t uidIndex;					// The first index in uid->uidByte[] that is used in the current Cascade Level.
	uint8_t currentLevelKnownBits;		// The number of known UID bits in the current Cascade Level.
	uint8_t buffer[9];					// The SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A
	uint8_t bufferUsed;				// The number of bytes used in the buffer, ie the number of bytes to transfer to the FIFO.
	uint8_t rxAlign;					// Used in BitFramingReg. Defines the bit position for the first bit received.
	uint8_t txLastBits;				// Used in BitFramingReg. The number of valid bits in the last transmitted byte.
	uint8_t *responseBuffer;
	uint8_t responseLength;

	// Description of buffer structure:
	//		Byte 0: SEL 				Indicates the Cascade Level: PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2 or PICC_CMD_SEL_CL3
	//		Byte 1: NVB					Number of Valid Bits (in complete command, not just the UID): High nibble: complete bytes, Low nibble: Extra bits.
	//		Byte 2: UID-data or CT		See explanation below. CT means Cascade Tag.
	//		Byte 3: UID-data
	//		Byte 4: UID-data
	//		Byte 5: UID-data
	//		Byte 6: BCC					Block Check Character - XOR of bytes 2-5
	//		Byte 7: CRC_A
	//		Byte 8: CRC_A
	// The BCC and CRC_A are only transmitted if we know all the UID bits of the current Cascade Level.
	//
	// Description of bytes 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: UID contents and cascade levels)
	//		UID size	Cascade level	Byte2	Byte3	Byte4	Byte5
	//		========	=============	=====	=====	=====	=====
	//		 4 bytes		1			uid0	uid1	uid2	uid3
	//		 7 bytes		1			CT		uid0	uid1	uid2
	//						2			uid3	uid4	uid5	uid6
	//		10 bytes		1			CT		uid0	uid1	uid2
	//						2			CT		uid3	uid4	uid5
	//						3			uid6	uid7	uid8	uid9

	// Sanity checks
	if (validBits > 80) {
		return 0;
	}

	uidComplete = false;
	while (!uidComplete) {
		// Set the Cascade Level in the SEL byte, find out if we need to use the Cascade Tag in byte 2.
		switch (cascadeLevel) {
		case 1:
			buffer[0] = PICC_CMD_SEL_CL1;
			uidIndex = 0;
			useCascadeTag = validBits && uid->size > 4;	// When we know that the UID has more than 4 bytes
			break;

		case 2:
			buffer[0] = PICC_CMD_SEL_CL2;
			uidIndex = 3;
			useCascadeTag = validBits && uid->size > 7;	// When we know that the UID has more than 7 bytes
			break;

		case 3:
			buffer[0] = PICC_CMD_SEL_CL3;
			uidIndex = 6;
			useCascadeTag = false;						// Never used in CL3.
			break;

		default:
			return STATUS_INTERNAL_ERROR;
			break;
		}

		// How many UID bits are known in this Cascade Level?
		currentLevelKnownBits = validBits - (8 * uidIndex);
		if (currentLevelKnownBits < 0) {
			currentLevelKnownBits = 0;
		}
		// Copy the known bits from uid->uidByte[] to buffer[]
		index = 2; // destination index in buffer[]
		if (useCascadeTag) {
			buffer[index++] = PICC_CMD_CT;
		}
		uint8_t bytesToCopy = currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 ? 1 : 0); // The number of bytes needed to represent the known bits for this level.
		if (bytesToCopy) {
			uint8_t maxBytes = useCascadeTag ? 3 : 4; // Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag
			if (bytesToCopy > maxBytes) {
				bytesToCopy = maxBytes;
			}
			for (count = 0; count < bytesToCopy; count++) {
				buffer[index++] = uid->uidByte[uidIndex + count];
			}
		}
		// Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits
		if (useCascadeTag) {
			currentLevelKnownBits += 8;
		}

		// Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
		selectDone = false;
		while (!selectDone) {
			// Find out how many bits and bytes to send and receive.
			if (currentLevelKnownBits >= 32) { // All UID bits in this Cascade Level are known. This is a SELECT.
				buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole bytes
				// Calculate BCC - Block Check Character
				buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
				// Calculate CRC_A
				result = CalculateCRC(buffer, 7, &buffer[7]);
				if (result != STATUS_OK) {
					return result;
				}
				txLastBits		= 0; // 0 => All 8 bits are valid.
				bufferUsed		= 9;
				// Store response in the last 3 bytes of buffer (BCC and CRC_A - not needed after tx)
				responseBuffer	= &buffer[6];
				responseLength	= 3;
			}
			else { // This is an ANTICOLLISION.
				txLastBits		= currentLevelKnownBits % 8;
				count			= currentLevelKnownBits / 8;	// Number of whole bytes in the UID part.
				index			= 2 + count;					// Number of whole bytes: SEL + NVB + UIDs
				buffer[1]		= (index << 4) + txLastBits;	// NVB - Number of Valid Bits
				bufferUsed		= index + (txLastBits ? 1 : 0);
				// Store response in the unused part of buffer
				responseBuffer	= &buffer[index];
				responseLength	= sizeof(buffer) - index;
			}

			// Set bit adjustments
			rxAlign = txLastBits;											// Having a separate variable is overkill. But it makes the next line easier to read.
			WriteRegister(BitFramingReg, (rxAlign << 4) + txLastBits);	// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

			// Transmit the buffer and receive the response.
			result = TransceiveData(buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign);
			if (result == STATUS_COLLISION) { // More than one PICC in the field => collision.
				uint8_t valueOfCollReg = ReadRegister(CollReg); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
				if (valueOfCollReg & 0x20) { // CollPosNotValid
					return STATUS_COLLISION; // Without a valid collision position we cannot continue
				}
				uint8_t collisionPos = valueOfCollReg & 0x1F; // Values 0-31, 0 means bit 32.
				if (collisionPos == 0) {
					collisionPos = 32;
				}
				if (collisionPos <= currentLevelKnownBits) { // No progress - should not happen
					return STATUS_INTERNAL_ERROR;
				}
				// Choose the PICC with the bit set.
				currentLevelKnownBits	= collisionPos;
				count			= currentLevelKnownBits % 8; // The bit to modify
				checkBit		= (currentLevelKnownBits - 1) % 8;
				index			= 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); // First byte is index 0.
				buffer[index]	|= (1 << checkBit);
			}
			else if (result != STATUS_OK) {
				return result;
			}
			else { // STATUS_OK
				if (currentLevelKnownBits >= 32) { // This was a SELECT.
					selectDone = true; // No more anticollision
					// We continue below outside the while.
				}
				else { // This was an ANTICOLLISION.
					// We now have all 32 bits of the UID in this Cascade Level
					currentLevelKnownBits = 32;
					// Run loop again to do the SELECT.
				}
			}
		} // End of while (!selectDone)

		// We do not check the CBB - it was constructed by us above.

		// Copy the found UID bytes from buffer[] to uid->uidByte[]
		index			= (buffer[2] == PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
		bytesToCopy		= (buffer[2] == PICC_CMD_CT) ? 3 : 4;
		for (count = 0; count < bytesToCopy; count++) {
			uid->uidByte[uidIndex + count] = buffer[index++];
		}

		// Check response SAK (Select Acknowledge)
		if (responseLength != 3 || txLastBits != 0) { // SAK must be exactly 24 bits (1 byte + CRC_A).
			return STATUS_ERROR;
		}
		// Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those bytes are not needed anymore.
		result = CalculateCRC(responseBuffer, 1, &buffer[2]);
		if (result != STATUS_OK) {
			return result;
		}
		if ((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2])) {
			return STATUS_CRC_WRONG;
		}
		if (responseBuffer[0] & 0x04) { // Cascade bit set - UID not complete yes
			cascadeLevel++;
		}
		else {
			uidComplete = true;
			uid->sak = responseBuffer[0];
		}
	} // End of while (!uidComplete)

	// Set correct uid->size
	uid->size = 3 * cascadeLevel + 1;

	return STATUS_OK;
}

uint8_t NTagRC522::SelectPICCCas2(Uid *uid, uint8_t validBits, uint8_t *response) {
	bool uidComplete, selectDone, useCascadeTag;
	uint8_t cascadeLevel = 2;
	uint8_t result;
	uint8_t count, checkBit, index;
	uint8_t uidIndex;					// The first index in uid->uidByte[] that is used in the current Cascade Level.
	uint8_t currentLevelKnownBits;		// The number of known UID bits in the current Cascade Level.
	uint8_t buffer[9];					// The SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A
	uint8_t bufferUsed;				// The number of bytes used in the buffer, ie the number of bytes to transfer to the FIFO.
	uint8_t rxAlign;					// Used in BitFramingReg. Defines the bit position for the first bit received.
	uint8_t txLastBits;				// Used in BitFramingReg. The number of valid bits in the last transmitted byte.
	uint8_t responseLen;

	// Description of buffer structure:
	//		Byte 0: SEL 				Indicates the Cascade Level: PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2 or PICC_CMD_SEL_CL3
	//		Byte 1: NVB					Number of Valid Bits (in complete command, not just the UID): High nibble: complete bytes, Low nibble: Extra bits.
	//		Byte 2: UID-data or CT		See explanation below. CT means Cascade Tag.
	//		Byte 3: UID-data
	//		Byte 4: UID-data
	//		Byte 5: UID-data
	//		Byte 6: BCC					Block Check Character - XOR of bytes 2-5
	//		Byte 7: CRC_A
	//		Byte 8: CRC_A
	// The BCC and CRC_A are only transmitted if we know all the UID bits of the current Cascade Level.
	//
	// Description of bytes 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: UID contents and cascade levels)
	//		UID size	Cascade level	Byte2	Byte3	Byte4	Byte5
	//		========	=============	=====	=====	=====	=====
	//		 4 bytes		1			uid0	uid1	uid2	uid3
	//		 7 bytes		1			CT		uid0	uid1	uid2
	//						2			uid3	uid4	uid5	uid6
	//		10 bytes		1			CT		uid0	uid1	uid2
	//						2			CT		uid3	uid4	uid5
	//						3			uid6	uid7	uid8	uid9

	// Sanity checks
	if (validBits > 80) {
		return 0;
	}

	buffer[0] = PICC_CMD_SEL_CL1;
	buffer[1] = 0x20;
	TransceiveData(buffer, 2, response, &responseLen, &validBits);
	uint8_t lastByte = 4;
	for(int i = 0; i < 4; i++)
		if(response[i] == 0) {
			lastByte = i;
			break;
		}

	//temporary. Deal with collisions later.
	if(lastByte != 4)
		return responseLen;

	uid->uidByte[0] = response[1];
	uid->uidByte[1] = response[2];
	uid->uidByte[2] = response[3];

	buffer[0] = PICC_CMD_SEL_CL1;
	buffer[1] = 0x70; //all bytes ok
	for(uint8_t i = 0; i < 5; i++)
		buffer[i+2] = response[i];

	//clear response
	for(uint8_t i = 0; i < responseLen; i++)
		response[i] = 0;

	CalculateCRC(buffer, 7, buffer+7);


	uint8_t err = TransceiveData(buffer, 9, response, &responseLen, &validBits);
	if(err != STATUS_OK)
		return err;
	//check for NAK
	if(!response[0])
		return STATUS_ERROR;

	//cascade level 2
	//clear response
	for(uint8_t i = 0; i < responseLen; i++)
		response[i] = 0;

	buffer[0] = PICC_CMD_SEL_CL2;
	buffer[1] = 0x20;

	TransceiveData(buffer, 2, response, &responseLen, &validBits);

	lastByte = 4;
	for(int i = 0; i < 4; i++)
		if(response[i] == 0) {
			lastByte = i;
			break;
		}
	//temporary. Deal with collisions later.
	if(lastByte != 4)
		return responseLen;

	uid->uidByte[3] = response[0];
	uid->uidByte[4] = response[1];
	uid->uidByte[5] = response[2];
	uid->uidByte[6] = response[3];

	buffer[0] = PICC_CMD_SEL_CL2;
	buffer[1] = 0x70; //all bytes ok
	for(uint8_t i = 0; i < 5; i++)
		buffer[i+2] = response[i];

	//clear response
	for(uint8_t i = 0; i < responseLen; i++)
		response[i] = 0;

	CalculateCRC(buffer, 7, buffer+7);

	err = TransceiveData(buffer, 9, response, &responseLen, &validBits);
	if(err != STATUS_OK)
		return err;
	//check for SAK = 0
	if(response[0])
		return STATUS_ERROR;


	// Set correct uid->size
	uid->size = 7;

	return STATUS_OK;
}

bool NTagRC522::IsCardPresent() {
	uint8_t AToReq[2];
	uint8_t bufferSize = sizeof(AToReq);

	// Reset baud rates
	WriteRegister(TxModeReg, 0x00);
	WriteRegister(RxModeReg, 0x00);
	// Reset ModWidthReg
	WriteRegister(ModWidthReg, 0x26);

	uint8_t result = WakeUpA(AToReq,&bufferSize);
	return !(result);
}

//perform soft reset
uint8_t NTagRC522::SoftReset() {
	uint8_t error = WriteRegister(CommandReg, ComSoftReset);
	osDelay(10); //might need adjusting
	//wait for reset bit to be cleared
	while((ReadRegister(CommandReg) & (1 << 4)))
		osDelay(1);
	return error;
}

//authenticate a sector on selected PICC.
//key is 8 bytes
uint8_t NTagRC522::Authenticate(uint8_t command, uint8_t blockAddr, Uid *uid, uint8_t *key) {
	uint8_t waitIRq = 0x10; // IdleIRq
	// Build command buffer
	uint8_t sendData[12];
	sendData[0] = command;
	sendData[1] = blockAddr;
	for (uint8_t i = 0; i < 6; i++) {	// 6 key bytes
		sendData[2+i] = key[i];
	}
	// Use the last uid bytes as specified in http://cache.nxp.com/documents/application_note/AN10927.pdf
	// section 3.2.5 "MIFARE Classic Authentication".
	// The only missed case is the MF1Sxxxx shortcut activation,
	// but it requires cascade tag (CT) byte, that is not part of uid.
	for (uint8_t i = 0; i < 4; i++) {				// The last 4 bytes of the UID
		sendData[8+i] = uid->uidByte[i+uid->size-4];
	}

	// Start the authentication.
	return CommunicateWithPICC(ComMFAuthent, waitIRq, &sendData[0], sizeof(sendData), nullptr, nullptr, nullptr);
}
//run after done communicating with a authenticated picc
uint8_t NTagRC522::StopCrypto() {
	ClearBitsRegister(Status2Reg, 0x08);
}


uint8_t NTagRC522::MIFARE_Read(uint8_t blockAddr, uint8_t *buffer, uint8_t *bufferSize) {
	buffer[0] = PICC_CMD_MF_READ;
	buffer[1] = blockAddr;

	CalculateCRC(buffer, 2, buffer+2);

	return TransceiveData(buffer, 4, buffer, bufferSize, nullptr, 0);
}

uint8_t NTagRC522::MIFARE_Write(uint8_t blockAddr, uint8_t *buffer, uint8_t *bufferSize) {
	uint8_t writeBuffer[30] = {};
	writeBuffer[0] = PICC_CMD_MF_WRITE;
	writeBuffer[1] = blockAddr;
	writeBuffer[3] = 0;
	writeBuffer[4] = 0;

	TransceiveData(writeBuffer, 4, writeBuffer, bufferSize, nullptr, 0);

	*bufferSize = 18;
	for(uint8_t i = 0; i < *bufferSize; i++)
		writeBuffer[i] = buffer[i];
	writeBuffer[16] = 0;
	writeBuffer[17] = 0;

	return TransceiveData(writeBuffer, 18, writeBuffer, bufferSize, nullptr, 0);
}

//reads and returns the version register value
uint8_t NTagRC522::SanityCheck() {
	return ReadRegister(this->VersionReg);
}
uint8_t NTagRC522::SanityCheck2() {
	//return this->ReadRegister(0x3C);
	return ReadRegister(0x2E);
}

uint8_t NTagRC522::GetWaterLevel() {
	return ReadRegister(WaterLevelReg);
}
uint8_t NTagRC522::SetWaterLevel(uint8_t set_value) {
	return WriteRegister(WaterLevelReg, set_value);
}

