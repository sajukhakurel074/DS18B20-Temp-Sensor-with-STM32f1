# ds18b20 Temperature Sensor in STM32f1

This library provides the API for Reading the respective temperatures of Multiple/Single DS18B20 Temperature Sensor/s connected in a single GPIO pin.

The example provided is configured for STM32F1 boards. For using the library for other MCU the GPIO Pin used for Data and Signal Transfer of DS18B20 has to be configured accordingly.
This can be achieved by changing the Header file of Library.

#define DS18B20_PORT GPIOB   		// need to change                                                                                                                                                                                                         
#define DS18B20_PIN GPIO_PIN_1      // need to change according to the requirement 

The library is based on the datasheet by Maxim Integrated Products. https://www.analog.com/media/en/technical-documentation/data-sheets/ds18b20.pdf 

Since the sensor works on the one wire communication protocol, data reading and signaling is done through a single wire, hence we need to work on the correct time period and understand the timing diagram for each functions.  
For this project I have used timer 2 (which is on APB1 Bus) provided by STM board is used to create the delays as per required. This timer is sourced by the internal clock of the MCU which in this case works on 72Mhz. The prescaler is set to 72-1 (since the counter starts from 0) giving 1 Mhz frequency (72 / 72Mhz = 1us). The counter is set to 1000 which results in range of our timer from 1us to 1ms (72 * 1000 / 72Mhz = 1ms). Hence creating a delay of 1us on each call.

For CRC calculation CRC-8-Maxim algotithm is used.


#Hardware Connections 
---------------------------------
|   PIN         | Connections   |
---------------------------------
|   Power       |   3V3         |

|   GND         |   GND         |

|   Data/Signal |   GPIO PIN    |
---------------------------------

