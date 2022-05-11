# STM32-Temp-Controlled-Fan
EE260 Embedded Systems Special Project: A temperature controlled fan using an STM32 microcontroller, DHT11 temperature sensor, L293D, and 5V external power supply. As the temperature increased, the fan speed would increase in steps using PWM.

All wiring is commented in the code and shown in setup.jpg and wiring.jpg

### Notes
The temperature sensor must be powered by the 5V on the microcontroller. It cannot share the power supply with the fan, as the fan will draw too much current and shut down the temperature sensor. 
