/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Aug 31, 2024
 *      Author: royal
 */


#include "stm32f407xx_gpio_driver.h"

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi ){

	if(EnorDi == ENABLE){

		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOB){
			GPIOC_PCLK_EN();
		}else if(pGPIOx == GPIOB){
			GPIOD_PCLK_EN();
		}else if(pGPIOx == GPIOB){
			GPIOE_PCLK_EN();
		}else if(pGPIOx == GPIOB){
			GPIOF_PCLK_EN();
		}else if(pGPIOx == GPIOB){
			GPIOG_PCLK_EN();
		}else if(pGPIOx == GPIOB){
			GPIOH_PCLK_EN();
		}else if(pGPIOx == GPIOB){
			GPIOI_PCLK_EN();
		}
	}
}

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

void GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
void GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);

void GPIO_ReadFromOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_ReadFromOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);
