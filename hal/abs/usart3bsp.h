/*************************************************************************************
*                                 < C/C++ HEADER FILE >                              *
**************************************************************************************/

/**
  ************************************************************************************
    @file     usart3bsp.h                                                            
    @author   Usuario	                                                              
    @version  v1.0.0                                                                  
    @date 	  23/08/2012                                                                 
    @brief                                                                            
                                                                                     
    @note
		 Copyright (C) ALTRIZ - Automação Industrial    
                                                     
  																					  
  ************************************************************************************
*/

#ifndef USART3BSP_H_
#define USART3BSP_H_

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

#define T_BUFFER_TX_SIZE_U3	8192
#define T_BUFFER_RX_SIZE_U3	512

/**************************************************************************************
*                           TYPEDEFS, CLASSES AND STRUCTURES                          *
***************************************************************************************/

/**************************************************************************************
*                                   GLOBAL VARIABLES                                  *
***************************************************************************************/

/**************************************************************************************
*                                   GLOBAL FUNCTIONS                                  *
***************************************************************************************/

void usart3bsp_Init(uint32_t baudrate);
void usart3bsp_DeInit(void);
void usart3bsp_Putc(char c);
void usart3bsp_PutString(const char *str);
uint32_t usart3bsp_SendNBytes(uint8_t *buff, uint16_t u16_size);
uint32_t usart3bsp_GetNBytes(uint8_t *buff, uint32_t u32_size);
void usart3bsp_ResetTxFIFO(void);
void usart3bsp_ResetRxFIFO(void);
void usart3bsp_SetRxCallback(void(*callback)(void));
void usart3bsp_SetTxCallback(void(*callback)(void));

#ifdef __cplusplus
}
#endif //__cplusplus

#endif /* USART3BSP_H_ */

/******************************* (C) COPYRIGHT ALTRIZ *********************************/

/*******************************      END OF FILE     *********************************/
