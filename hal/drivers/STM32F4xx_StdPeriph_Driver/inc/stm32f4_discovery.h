/**
  ******************************************************************************
  * @file    stm32f4_discovery.h
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    28-October-2011
  * @brief   This file contains definitions for STM32F4-Discovery Kit's Leds and 
  *          push-button hardware resources.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************  
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4_DISCOVERY_H
#define __STM32F4_DISCOVERY_H

#ifdef __cplusplus
 extern "C" {
#endif
                                              
/* Includes ------------------------------------------------------------------*/
 #include "stm32f4xx.h"
   
/** @addtogroup Utilities
  * @{
  */
  
/** @addtogroup STM32F4_DISCOVERY
  * @{
  */
      
/** @addtogroup STM32F4_DISCOVERY_LOW_LEVEL
  * @{
  */ 

/** @defgroup STM32F4_DISCOVERY_LOW_LEVEL_Exported_Types
  * @{
  */
typedef enum 
{
  LED4 = 0,
  LED3 = 1,
  LED5 = 2,
  LED6 = 3,
  RELAY_1BP = 4,
  RELAY_2BP = 5,
  RELAY_1BN = 6,
  RELAY_2BN = 7
} Led_TypeDef;

typedef enum 
{  
  BUTTON_USER = 0,
  SENSOR_ED1,
  SENSOR_ED2
} Button_TypeDef;

typedef enum 
{  
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1
} ButtonMode_TypeDef;

typedef enum
{
	SENSOR_MODE_GPIO = 0,
	SENSOR_MODE_EXTI = 1
}SensorMode_TypeDef;

/**
  * @}
  */ 

/** @defgroup STM32F4_DISCOVERY_LOW_LEVEL_Exported_Constants
  * @{
  */ 

/** @addtogroup STM32F4_DISCOVERY_LOW_LEVEL_LED
  * @{
  */
#define LEDn                             4

#define LED4_PIN                         GPIO_Pin_12
#define LED4_GPIO_PORT                   GPIOD
#define LED4_GPIO_CLK                    RCC_AHB1Periph_GPIOD  
  
#define LED3_PIN                         GPIO_Pin_13
#define LED3_GPIO_PORT                   GPIOD
#define LED3_GPIO_CLK                    RCC_AHB1Periph_GPIOD  
  
#define LED5_PIN                         GPIO_Pin_14
#define LED5_GPIO_PORT                   GPIOD
#define LED5_GPIO_CLK                    RCC_AHB1Periph_GPIOD  
  
#define LED6_PIN                         GPIO_Pin_15
#define LED6_GPIO_PORT                   GPIOD
#define LED6_GPIO_CLK                    RCC_AHB1Periph_GPIOD

#define RELAY_1_B_POS_PIN                     GPIO_Pin_13
#define RELAY_1_B_POS_GPIO_PORT               GPIOB
#define RELAY_1_B_POS_GPIO_CLK                RCC_AHB1Periph_GPIOB

#define RELAY_2_B_POS_PIN                     GPIO_Pin_15
#define RELAY_2_B_POS_GPIO_PORT               GPIOB
#define RELAY_2_B_POS_GPIO_CLK                RCC_AHB1Periph_GPIOB

#define RELAY_1_B_NEG_PIN                     GPIO_Pin_9
#define RELAY_1_B_NEG_GPIO_PORT               GPIOD
#define RELAY_1_B_NEG_GPIO_CLK                RCC_AHB1Periph_GPIOD

#define RELAY_2_B_NEG_PIN                     GPIO_Pin_11
#define RELAY_2_B_NEG_GPIO_PORT               GPIOD
#define RELAY_2_B_NEG_GPIO_CLK                RCC_AHB1Periph_GPIOD

#define ED1_PIN		                     GPIO_Pin_15
#define ED1_GPIO_PORT		             GPIOB
#define ED1_GPIO_CLK        		     RCC_AHB1Periph_GPIOD

#define ED2_PIN		                     GPIO_Pin_11
#define ED2_GPIO_PORT		             GPIOD
#define ED2_GPIO_CLK        		     RCC_AHB1Periph_GPIOD

/**
  * @}
  */ 
  
/** @addtogroup STM32F4_DISCOVERY_LOW_LEVEL_BUTTON
  * @{
  */  
#define BUTTONn                          1  

/**
 * @brief Wakeup push-button
 */
#define USER_BUTTON_PIN                GPIO_Pin_0
#define USER_BUTTON_GPIO_PORT          GPIOA
#define USER_BUTTON_GPIO_CLK           RCC_AHB1Periph_GPIOA
#define USER_BUTTON_EXTI_LINE          EXTI_Line0
#define USER_BUTTON_EXTI_PORT_SOURCE   EXTI_PortSourceGPIOA
#define USER_BUTTON_EXTI_PIN_SOURCE    EXTI_PinSource0
#define USER_BUTTON_EXTI_IRQn          EXTI0_IRQn 
/**
  * @}
  */ 
  
/** @defgroup STM32F4_DISCOVERY_LOW_LEVEL_Exported_Macros
  * @{
  */  
/**
  * @}
  */ 


/** @defgroup STM32F4_DISCOVERY_LOW_LEVEL_Exported_Functions
  * @{
  */
void STM_EVAL_LEDInit(Led_TypeDef Led);
void STM_EVAL_GFDRelayInit(Led_TypeDef Relay);
void STM_EVAL_LEDOn(Led_TypeDef Led);
void STM_EVAL_LEDOff(Led_TypeDef Led);
void STM_EVAL_GFDRelayOn(Led_TypeDef Relay);
void STM_EVAL_GFDRelayOff(Led_TypeDef Relay);
void STM_EVAL_LEDToggle(Led_TypeDef Led);
void STM_EVAL_PBInit(Button_TypeDef Button, ButtonMode_TypeDef Button_Mode);
void STM_EVAL_GFDEDSensorInit(Button_TypeDef Sensor, SensorMode_TypeDef Sensor_Mode);
uint32_t STM_EVAL_PBGetState(Button_TypeDef Button);
uint32_t STM_EVAL_GFDEDSensorGetState(Button_TypeDef Sensor);

/**
  * @}
  */
  
#ifdef __cplusplus
}
#endif

#endif /* __STM32F4_DISCOVERY_H */
/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */

 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
