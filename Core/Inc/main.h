/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
		//-----------------------------BluePill pins--------------------------------------
	
// 1 (VBAT): not used
// 2 (PC13): Indication LED
// 3 (PC14): not used
// 4 (PC15): not used
// 5 (PA0): ADC chanel 3.3
// 6 (PA1): ADC chanel 5
// 7 (PA2): ADC chanel 5
// 8 (PA3): ADC chanel 12
// 9 (PA4): ADC chanel 18
//10 (PA5): ADC chanel 24
//11 (PA6): ADC chanel 30
//12 (PA7): Master Reset 595, when it has high level, register enable
//13 (PB0): SRCLK 595. Clock pin. Bit is entered when a low to high 
//14 (PB1): RCLK 595. Latch pin
//15 (PB10): DataPin 595.
//16 (PB11): Sound pin. Sound on when it has high level
//17 (NRST): not used
//18 (+3.3V): +3.3V
//19 (GND): GND
//20 (GND): GND
//21 (PB12): not used
//22 (PB13): Datapin 164 register, LED on front panel
//23 (PB14): Clockpin 164 register
//24 (PB15): not used
//25 (PA8):  Sound pin. Sound on when it has high level
//26 (PA9): TXd
//27 (PA10): RXd
//28 (PA11): Read state button 405 multiplex
//29 (PA12): not used
//30 (PA15): not used
//31 (PB3): S2
//32 (PB4): S1
//33 (PB5): S0
//34 (PB6): Datatemp ds18b20
//35 (PB7): Interrupt button, interrupt on low level
//36 (PB8): CLK tm1637
//37 (PB9): DATA tm1637
//38 (+5V): +5V
//39 (GND): GND
//40 (+3.3V): +3.3V

	//----------------------------------------------------------------------------------
	//----------------------------------------------------------------------------------
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_tim.h"
#include "stm32f1xx_ll_usart.h"
#include "stm32f1xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f1xx_HAL_flash.h"
#include <stdio.h>
#include "delay_micros.h"
#include "tm1637.h"
#include "button.h"
#include "FlashPROM.h"
#include "ds18b20.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void ButtonReadInterrupt(void);
void Long_sound(void);
void Short_sound(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin LL_GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define Ch1_Pin LL_GPIO_PIN_0
#define Ch1_GPIO_Port GPIOA
#define Ch2_Pin LL_GPIO_PIN_1
#define Ch2_GPIO_Port GPIOA
#define Ch3_Pin LL_GPIO_PIN_2
#define Ch3_GPIO_Port GPIOA
#define Ch4_Pin LL_GPIO_PIN_3
#define Ch4_GPIO_Port GPIOA
#define Ch5_Pin LL_GPIO_PIN_4
#define Ch5_GPIO_Port GPIOA
#define Ch6_Pin LL_GPIO_PIN_5
#define Ch6_GPIO_Port GPIOA
#define Ch7_Pin LL_GPIO_PIN_6
#define Ch7_GPIO_Port GPIOA
#define MasterReset595_Pin LL_GPIO_PIN_7
#define MasterReset595_GPIO_Port GPIOA
#define ClockPin595_Pin LL_GPIO_PIN_0
#define ClockPin595_GPIO_Port GPIOB
#define latchPin595_Pin LL_GPIO_PIN_1
#define latchPin595_GPIO_Port GPIOB
#define DataPin595_Pin LL_GPIO_PIN_10
#define DataPin595_GPIO_Port GPIOB
#define dataled_Pin LL_GPIO_PIN_13
#define dataled_GPIO_Port GPIOB
#define clockled_Pin LL_GPIO_PIN_14
#define clockled_GPIO_Port GPIOB
#define Sound_Pin LL_GPIO_PIN_8
#define Sound_GPIO_Port GPIOA
#define ReadButton_Pin LL_GPIO_PIN_11
#define ReadButton_GPIO_Port GPIOA
#define CS_Pin LL_GPIO_PIN_15
#define CS_GPIO_Port GPIOA
#define S2_Pin LL_GPIO_PIN_3
#define S2_GPIO_Port GPIOB
#define S1_Pin LL_GPIO_PIN_4
#define S1_GPIO_Port GPIOB
#define S0_Pin LL_GPIO_PIN_5
#define S0_GPIO_Port GPIOB
#define Datatemp_Pin LL_GPIO_PIN_6
#define Datatemp_GPIO_Port GPIOB
#define interrupt_Pin LL_GPIO_PIN_7
#define interrupt_GPIO_Port GPIOB
#define interrupt_EXTI_IRQn EXTI9_5_IRQn
#define CLK_Pin LL_GPIO_PIN_8
#define CLK_GPIO_Port GPIOB
#define DIO_Pin LL_GPIO_PIN_9
#define DIO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define Sound_Off LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_11);
#define Sound_On LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_11);
#define Sample_ADC 2
#define Shortsoundtime 2
#define Longsoundtime 4



/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
