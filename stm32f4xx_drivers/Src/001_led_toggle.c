/*
 * 001_led_toggle.c
 *
 *  Created on: Sep 1, 2024
 *      Author: royal
 */


#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

void delay(void){
	for	(uint32_t i = 0; i < 500000/2; i++);
}

int main(void){

	GPIO_Handle_t GpioLED;

	GpioLED.pGPIOx = GPIOD;
	GpioLED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GpioLED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GpioLED);

	while(1){

		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		delay();
	}
	return 0;
}
