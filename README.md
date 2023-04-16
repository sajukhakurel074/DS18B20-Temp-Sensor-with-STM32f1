# ds18b20 Temperature Sensor in STM32f1

This library provides the API for Reading the respective temperatures of Multiple/Single DS18B20 Temperature Sensor/s connected in a single GPIO pin.

The example provided is configured for STM32F1 boards. For using the library for other MCU the GPIO Pin used for Data and Signal Transfer of DS18B20 has to be configured accordingly.
This can be achieved by changing the Header file of Library.

#define DS18B20_PORT GPIOB   		// need to change
#define DS18B20_PIN GPIO_PIN_1      // need to change according requirement 

