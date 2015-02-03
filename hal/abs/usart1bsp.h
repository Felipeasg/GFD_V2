/*************************************************************************************
*                                 < C/C++ HEADER FILE >                              *
**************************************************************************************/

/**
  ************************************************************************************
    @file     usart1bsp.h                                                            
    @author   Usuario	                                                              
    @version  v1.0.0                                                                  
    @date 	  16/08/2012                                                                 
    @brief                                                                            
                                                                                     
    @note
		 Copyright (C) ALTRIZ - Automação Industrial    
                                                     
  																					  
  ************************************************************************************
*/

#ifndef USART1BSP_H_
#define USART1BSP_H_

#ifdef __cplusplus
extern "C"
{
#endif //__cplusplus

/**************************************************************************************
*                                       INCLUDES                                      *
***************************************************************************************/

#include <stdint.h>
#include <stdbool.h>

/**************************************************************************************
*                                DEFINITIONS AND MACROS                               *
***************************************************************************************/

#define T_BUFFER_TX_SIZE_U1	8192
#define T_BUFFER_RX_SIZE_U1	512

/**************************************************************************************
*                           TYPEDEFS, CLASSES AND STRUCTURES                          *
***************************************************************************************/

/**************************************************************************************
*                                   GLOBAL VARIABLES                                  *
***************************************************************************************/

void usart1bsp_Init(uint32_t baudrate);
void usart1bsp_DeInit(void);
void usart1bsp_Putc(char c);
void usar1tbsp_PutString(const char *str);
void usart1bsp_SendBuffer(const uint8_t *buff, uint32_t u32_size);
uint32_t usart1bsp_SendNBytes(uint8_t *buff, uint16_t u16_size);
uint32_t usart1bsp_GetNBytes(uint8_t *buff, uint32_t u32_size);
void usart1bsp_ResetTxFIFO(void);
void usart1bsp_ResetRxFIFO(void);
void usart1bsp_SetRxCallback(void(*callback)(void));
void usart1bsp_SetTxCallback(void(*callback)(void));

/**************************************************************************************
*                                   GLOBAL FUNCTIONS                                  *
***************************************************************************************/

void usart1bsp_Init(uint32_t u32_baudrate);
void usart1bsp_Putc(char c);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif /* USART1BSP_H_ */

/******************************* (C) COPYRIGHT ALTRIZ *********************************/

/*******************************      END OF FILE     *********************************/
