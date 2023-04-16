/*
 * 18B20.c
 *
 *  Created on: Jan 31, 2023
 *      Author: Saju Khakurel
 */

#include "18B20.h"
#include "string.h"

static int Presence;
//float Temperature;
static uint16_t TEMP;
//static uint32_t ROM_id1, ROM_id2;
static uint64_t ROM_id[MAX_DEVICE];
static uint64_t ROM_alarm_id[MAX_DEVICE];
static uint8_t new_rom_id[8];
static uint8_t bit_id, bit_id_comp;
static uint8_t search_value;
static uint8_t bit_number, discrepancy_marker, last_discrepancy;
static uint8_t FLAG_DONE;
uint8_t num;
uint8_t count = 0;
static uint8_t counts = 0;
static uint8_t bit_counter = 0;

extern TIM_HandleTypeDef htim2;

int Temp_init(void) {
	printf("\n\n\n\n\n\nFrom TEMP Sensor Test\r\n");
	HAL_TIM_Base_Start(&htim2);  // Start the timer for getting the delay as required

	Presence = DS18B20_Start();
	if (Presence != 1) {
		printf("Presence not detected\n");
		return 0;
	}
	return 1;
}

int DS18B20_Start(void) {
	int Response = 0;
	Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);   // set the pin as output
	HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, 0); // pull the pin low
	delay(480);   // delay according to data sheet

	Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);    // set the pin as input
	delay(80);    // delay according to data sheet

	if (!(HAL_GPIO_ReadPin(DS18B20_PORT, DS18B20_PIN)))
		Response = 1; // if the pin is low i.e the presence pulse is detected
	else
		Response = -1;

	delay(400); // Waiting to complete the response cycle

	return Response;
}

void DS18B20_Write(uint8_t data, uint8_t bit) {
	int loop = 0;
	if (bit == 1) {
		loop = 1; // Bit write
	} else {
		loop = 8; // Byte write
	}
	Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);  // set as output

	for (int i = 0; i < loop; i++) {

		if ((data & (1 << i)) != 0)  // if the bit is high
				{
			// write 1

			Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);  // set as output
			HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, 0); // pull the pin LOW
			delay(1);  // wait for 1 us

			Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);  // set as input
			delay(60);  // wait for 60 us
		}

		else  // if the bit is low
		{
			// write 0

			Set_Pin_Output(DS18B20_PORT, DS18B20_PIN); // set as output
			HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, 0); // pull the pin LOW
			delay(60);  // wait for 60 us

			Set_Pin_Input(DS18B20_PORT, DS18B20_PIN); // set as input
		}
	}
}

uint8_t DS18B20_Read(uint8_t bit) {
	int loop = 0;
	if (bit == 1) {  // Bit read
		loop = 1;
	} else {
		loop = 8; // Byte read
	}

	uint8_t value = 0;
	Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);

	for (int i = 0; i < loop; i++) {
		Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);   // set as output

		HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, 0); // pull the data pin LOW
		delay(5);  // wait for 5 us

		Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);  // set as input

		if (HAL_GPIO_ReadPin(DS18B20_PORT, DS18B20_PIN))  // if the pin is HIGH store 1 in current index by shifting
				{
			value |= 1 << i;  // read = 1
		}
		delay(60);  // wait for 60 us
	}
	return value;
}

int Search_ROM(int cmd) {

	Presence = DS18B20_Start(); // Before search, make sure the sensor is detectable
	if (Presence != 1) {
		printf("Presence not detected\n");
		return 0;
	}

	if (FLAG_DONE == SET) {	// if all devices are detected, the flag is SET
		return 0;
	}
	HAL_Delay(1);

	bit_number = 1;
	counts = 0;
	discrepancy_marker = 0;
	DS18B20_Write(cmd, 0);  // Send ROM command
	bit_counter = 0;

	do {

		bit_id = DS18B20_Read(1);				// read LSB bit value
		bit_id_comp = DS18B20_Read(1);  // read LSB bit value complement

		if (bit_id && bit_id_comp) { // 11 is the case for false value indicating no more devices or faulty case
			printf("No more devices\n");
			return 0;
		} else {
			if (bit_id == bit_id_comp) // 00 indicates both 0 and 1 bit value at LSB of available devices
					{
				if (bit_number == last_discrepancy) {
					search_value = 1;
				} else {
					if (bit_number > last_discrepancy) {
						search_value = 0;
						discrepancy_marker = bit_number;

					} else {
						if (search_value == 0) {
							discrepancy_marker = bit_number;
						}

					}
				}

			} else { // this indicates same 0 or 1 value at LSB of available devices
				search_value = bit_id;   // setting either 0 or 1 search
			}
			DS18B20_Write(search_value, 1);	// Selecting the devices having ongoing-LSB value as search value (0 or 1), After this step only the device/s with search value response in next read
			new_rom_id[counts] |= search_value << bit_counter; // store the read value in current index position of ID

			if (bit_number % 8 == 0) {		// increase counts only after 1 byte is read
				counts++;

			}
			bit_counter++;	// indexing of bit position on 1 byte
			if (bit_counter >= 8) {
				bit_counter = 0;
			}

		}

		bit_number++;		// increased in every loop

	} while (bit_number < 65);  // ROM id is 64 bit

	last_discrepancy = discrepancy_marker; // Saving the last discrepancy position for non-repeating same id

	if (last_discrepancy == 0) {
		FLAG_DONE = SET;
	}
	count = count + 1;

	return 1;
}

void Match_ROM(int device) {

	DS18B20_Write(MATCH_ROM, 0);
	for (int i = 0; i < 8; i++) {
		DS18B20_Write(((uint8_t*) &ROM_id[device - 1])[i], 0);
	}

}

void Read_Temp(int select) {

	uint8_t data[9] = { 0 };
	int neg = 0;
	int bit8_value;
	float decimal;

	Presence = DS18B20_Start();
	if (Presence != 1) {
		printf("Presence not detected for reading temp\n");
		return;
	}

	Match_ROM(select);					// Only the device with selected ROM ID will respond to the next commands
	DS18B20_Write(CONVERT_S, 0);		// Convert T
	HAL_Delay(10);						// Delay for Conversion
	Presence = DS18B20_Start();			// Reset is required after convert command

	Match_ROM(select);				// Only the device with selected ROM ID will respond to the next commands
	DS18B20_Write(READ_S, 0);		// Read Scratch pad

	for (int i = 0; i < 9; i++) {
		data[i] = DS18B20_Read(0);
	}

	if (data[8] != CRC_CHECK(data, 8)) {	// Check read data integrity
		printf("CRC Check Failed\n");
		return;
	}

	TEMP = (data[1] << 8) | data[0];	// converted tempetaure is store in two 8 bits registers

	if (TEMP & 0x8000)   //check of the temperature is negative
			{
		printf("Temperature is in -ve\n");
		TEMP = ~TEMP + 1;       // 2's complement in case of -ve temperature
		neg = 1;				// Set -ve temperature flag
	}

	bit8_value = TEMP >> 4;
	bit8_value |= ((TEMP >> 8) & 0x7) << 4;

	switch (data[4]) {
	case 31:
		decimal = (TEMP >> 3) & 0x01;
		decimal = decimal / 2;
		break;
	case 63:
		decimal = (TEMP >> 2) & 0x03;
		decimal = decimal / 4;
		break;
	case 95:
		decimal = (TEMP >> 1) & 0x07;
		decimal = decimal / 8;
		break;
	case 127:
		decimal = TEMP & 0x0F;
		decimal = decimal / 16;
		break;
	default:
		break;
	}

	decimal = bit8_value + decimal;
	if (neg) {
		decimal = 0 - decimal;    // negate the temperature if is -ve;
	}
	printf("Temperature of device %d = %f \n", select, decimal);
}

void Find_Temp_devices() {
	last_discrepancy = 0;
	while (Search_ROM(SEARCH_ROM)) {

		memcpy((uint8_t*) &ROM_id[count - 1], new_rom_id, sizeof(new_rom_id));
		printf("Room id of the sensor= %d\n{ ", count);
		for (int i = 0; i < 8; i++) {
			printf("0x%x ", ((uint8_t*) &ROM_id[count - 1])[i]);
		}
		printf("}\n\n");

		if (FLAG_DONE == 1) {
			break;
		}
		memset(new_rom_id, 0, sizeof(new_rom_id));
		if(((uint8_t*) &ROM_id[count - 1])[0] !=  DS18B20_FAMILY_CODE)
		{
			printf("Wrong value read \n");
			break;
		}
	}


}

void Set_Threshold(int upper, int lower, int device) {
	int TH, TL, Resolution;

	Presence = DS18B20_Start();
	if (Presence != 1) {
		printf("Presence not detected for setting Thresholds\n");
		return;
	}

	Match_ROM(device);
	DS18B20_Write(READ_S, 0);		// Read Scratch pad

	DS18B20_Read(0);  // Skip first two bytes
	DS18B20_Read(0);

	TH = DS18B20_Read(0);
	TL = DS18B20_Read(0);
	Resolution = DS18B20_Read(0);

	printf("Before Set Upper TH of device %d = %d\n", device, TH);
	printf("Before Set Lower TL of device %d = %d\n", device, TL);
	printf("Before Set Config = %d\n", Resolution);

	if (upper > 125) { // Set possible maximum and minimum values if set value exceeds limit
		upper = 125;
	}
	if (lower < -55) {
		lower = -55;
	}

	Presence = DS18B20_Start();
	if (Presence != 1) {
		printf("Presence not detected for setting Thresholds\n");
		return;
	}

	Match_ROM(device);
	DS18B20_Write(WRITE_S, 0);		// Write Scratch pad

	DS18B20_Write(upper, 0);		// Write upper threshold Scratch pad
	DS18B20_Write(lower, 0);		// Write lower threshold  Scratch pad
	DS18B20_Write(Resolution, 0);		// Write config Scratch pad
}

void Check_Alarm() {
	last_discrepancy = 0;
	count = 0;
	FLAG_DONE = RESET;
	while (Search_ROM(ALARM_ROM)) {

		memcpy((uint8_t*) &ROM_alarm_id[count - 1], new_rom_id,
				sizeof(new_rom_id));
		printf("Room id of the sensor= %d\n{ ", count);
		for (int i = 0; i < 8; i++) {
			printf("0x%x ", ((uint8_t*) &ROM_alarm_id[count - 1])[i]);
		}
		printf("}\n\n");

		if (FLAG_DONE == 1) {
			break;
		}
		memset(new_rom_id, 0, sizeof(new_rom_id));

	}

}

uint8_t CRC_CHECK(uint8_t *data, int len) {
	uint8_t crc = 0, inbyte, i, mix;

	while (len--) {
		inbyte = *data++;
		for (i = 8; i; i--) {
			mix = (crc ^ inbyte) & 0x01;
			crc >>= 1;
			if (mix) {
				crc ^= 0x8C;
			}
			inbyte >>= 1;
		}
	}

	/* Return calculated CRC */
	return crc;
}
void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void delay(uint32_t delay) {
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	while (__HAL_TIM_GET_COUNTER(&htim2) < delay) {

	}

}

