/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Aug 31, 2024
 *      Author: royal
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include<stm32f407xx.h>


typedef struct
{
	uint16_t GPIO_PinNumber;
	uint16_t GPIO_PinMode;
	uint16_t GPIO_PinSpeed;
	uint16_t GPIO_PinPuPdControl;
	uint16_t GPIO_PinOPType;
	uint16_t GPIO_PinAltFunMode;

}GPIO_PinConfig_t;


typedef struct
{
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi );

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

void GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
void GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);

void GPIO_ReadFromOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_ReadFromOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
