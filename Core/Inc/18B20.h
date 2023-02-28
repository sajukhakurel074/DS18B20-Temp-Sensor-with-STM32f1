/*
 * 18B20.h
 *
 *  Created on: Jan 31, 2023
 *      Author: Saju Khakurel
 */

#ifndef INC_18B20_H_
#define INC_18B20_H_

#include "main.h"

#define MAX_DEVICE 5
#define UPPER_TH 30
#define LOWER_TH 5

#define SEARCH_ROM		0xF0
#define READ_ROM		0x33
#define MATCH_ROM 		0x55
#define SKIP_ROM		0xCC
#define ALARM_ROM		0xEC
#define CONVERT_S		0x44
#define WRITE_S			0x4E
#define READ_S			0xBE
#define COPY_S			0x48
#define RECALL_S		0xB8
#define READ_POWER		0xB4

#define DS18B20_PORT GPIOB   		// need to change
#define DS18B20_PIN GPIO_PIN_1      // need to change according requirement

#define DS18B20_FAMILY_CODE             0x28

int Temp_init(void);
int DS18B20_Start(void);
void DS18B20_Write(uint8_t data, uint8_t bit);
uint8_t DS18B20_Read(uint8_t bit);
void Find_Temp_devices();
void Match_ROM(int device);
void Read_Temp(int select);
void Set_Threshold(int upper, int lower, int device);
void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
int Search_ROM(int cmd);
void Check_Alarm(void);
uint8_t CRC_CHECK(uint8_t *data, int len);
void delay(uint32_t delay);


#endif /* INC_18B20_H_ */
