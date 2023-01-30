/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DS18B20_PORT GPIOB
#define DS18B20_PIN GPIO_PIN_1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t Temp_byte1;
uint8_t Temp_byte2;

int Presence;
float Temperature;
uint16_t TEMP;

uint32_t ROM_id1, ROM_id2;
//uint64_t ROM_id;

uint64_t ROM_id[10];
uint8_t new_rom_id[8];
uint8_t bit_id, bit_id_comp;
uint8_t search_value;
uint8_t bit_number, discrepancy_marker, last_discrepancy;

uint8_t FLAG_DONE;
uint8_t count = 0;
uint8_t counts = 0;
uint8_t bit_counter = 0;
//uint8_t device_number=0;
//int last_zero;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
int Temp_init();

int DS18B20_Start(void);
void DS18B20_Write(uint8_t data, uint8_t bit);
uint8_t DS18B20_Read(uint8_t bit);

void Find_Temp_devices();
void Match_ROM(int device);
void Read_Temp(int select);

void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

int Search_ROM();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay(uint32_t delay) {
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	while (__HAL_TIM_GET_COUNTER(&htim2) < delay) {

	}

}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART1_UART_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	if (Temp_init() != 1) {
		printf("Temp Sensor Initialization failed\n");
	}

	Find_Temp_devices();
	printf("Number of devices on bus = %u\n", count);

	while (1) {
		printf("\n\n");
		Read_Temp(1);
		Read_Temp(2);
		HAL_Delay(1000);

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 72 - 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 1000;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

	/*Configure GPIO pin : PC13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PB1 */
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
#ifdef __GNUC__

int __io_putchar(int ch)
#else
int fputc(int ch, FILE *f)
#endif
{
	HAL_UART_Transmit(&huart1, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	return ch;
}

int Temp_init() {
	printf("\n\n\n\n\n\nFrom TEMP Sensor Test\r\n");
	HAL_TIM_Base_Start(&htim2);  // Start the timer

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

			Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);
			HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, 0); // pull the pin LOW
			delay(60);  // wait for 60 us

			Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);
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
		if (HAL_GPIO_ReadPin(DS18B20_PORT, DS18B20_PIN))  // if the pin is HIGH
				{
			value |= 1 << i;  // read = 1
		}
		delay(60);  // wait for 60 us
	}
	return value;
}

int Search_ROM() {

	Presence = DS18B20_Start();
	if (Presence != 1) {
		printf("Presence not detected\n");
		return 0;
	}

	if (FLAG_DONE == SET) {
		return 0;
	}
	HAL_Delay(1);

	bit_number = 1;
	counts = 0;
	discrepancy_marker = 0;
	DS18B20_Write(0xF0, 0);  // Send Search ROM command
	bit_counter = 0;

	do {

		bit_id = DS18B20_Read(1);				// read LSB bit value
		bit_id_comp = DS18B20_Read(1);  // read LSB bit value complement

		if (bit_id && bit_id_comp) { // 11 is the case for false value indicating no more devices
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
			DS18B20_Write(search_value, 1);	// Selecting the devices having ongoing-LSB value as search value (0 or 1)
			new_rom_id[counts] |= search_value << bit_counter;

			if (bit_number % 8 == 0) {
				counts++;

			}
			bit_counter++;
			if (bit_counter >= 8) {
				bit_counter = 0;
			}

		}

		bit_number++;

	} while (bit_number < 65);

	last_discrepancy = discrepancy_marker;

	if (last_discrepancy == 0) {
		FLAG_DONE = SET;
//		printf("Done Flag is SET\n");
	}
	count = count + 1;

	return 1;
}

void Match_ROM(int device) {

	DS18B20_Write(0x55, 0);
	for (int i = 0; i < 8; i++) {
		DS18B20_Write(((uint8_t*) &ROM_id[device - 1])[i], 0);
	}

}

void Read_Temp(int select) {
	Presence = DS18B20_Start();
	if (Presence != 1) {
		printf("Presence not detected\n");
	}

	Match_ROM(select);
	DS18B20_Write(0x44, 0);		// Convert T
	Presence = DS18B20_Start();

	Match_ROM(select);
	DS18B20_Write(0xBE, 0);		// Read Scratch pad
	uint8_t data[9] = { 0 };

	for (int i = 0; i < 9; i++) {
		data[i] = DS18B20_Read(0);
	}

	TEMP = (data[1] << 8) | data[0];
	Temperature = (float) TEMP / 16;

	printf("Temperature of device %d = %f \n", select, Temperature);
}

void Find_Temp_devices() {
	last_discrepancy = 0;
	while (Search_ROM()) {

		memcpy((uint8_t*) &ROM_id[count - 1], new_rom_id, sizeof(new_rom_id));
//		printf("\n\n");
		printf("Room id of the sensor= %d\n{ ", count);
		for (int i = 0; i < 8; i++) {
			printf("0x%x ", ((uint8_t*) &ROM_id[count - 1])[i]);
		}
		printf("}\n\n");

		if (FLAG_DONE == 1) {
			break;
		}
		memset(new_rom_id, 0, sizeof(new_rom_id));

	}

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

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	printf("From the error handler\n");
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
