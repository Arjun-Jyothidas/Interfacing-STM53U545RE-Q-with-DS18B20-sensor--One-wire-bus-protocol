# Interfacing-STM53U545RE-Q-with-DS18B20-sensor--One-wire-bus-protocol
Following repository contains HAL C code for developing a one-wire bus communication between STM32U545RE-Q and DS18B20 sensor.
DS18B20 sensor is a temperature sensor. The details of the sensor are provided in the datasheet with which the sensor is interfaced to the microcontroller. The link for the datasheet is given as:
[https://www.mouser.com/datasheet/2/758/DHT11-Technical-Data-Sheet-Translated-Version-1143054.pdf](https://cdn.sparkfun.com/datasheets/Sensors/Temp/DS18B20.pdf)
. Only the main source files are added in this repository as the rest of the files are generated automatically by the STMCubeIDE.
The target microcontroller - STM32U545RE-Q is selected in the STMCubeMX, matching the hardware.
The following repository shows that the DS18B20 sensor communicating with the microcontroller and showing the temperature at real time conditions.
The resulting temperature readings are displayed using Serial console through UART protocol.
