/**
*****************************************************************************
**
**  File        : stm32f4xx_it.c
**
**  Abstract    : Main Interrupt Service Routines.
**                This file provides template for all exceptions handler and
**                peripherals interrupt service routine.
**
**  Environment : Atollic TrueSTUDIO(R)
**                STMicroelectronics STM32F4xx Standard Peripherals Library
**
**  Distribution: The file is distributed “as is,” without any warranty
**                of any kind.
**
**  (c)Copyright Atollic AB.
**  You may use this file as-is or modify it according to the needs of your
**  project. Distribution of this file (unmodified or modified) is not
**  permitted. Atollic AB permit registered Atollic TrueSTUDIO(R) users the
**  rights to distribute the assembled, compiled & linked contents of this
**  file as part of an application binary file, provided that it is built
**  using the Atollic TrueSTUDIO(R) toolchain.
**
**
*****************************************************************************
*/

/* Includes ------------------------------------------------------------------*/

#include <stdbool.h>

#include "stm32f4xx_it.h"
#include "stm32f4xx_dma.h"
#include "scheduler.h"
#include "iprintf.h"
#include "usb_core.h"
#include "usbd_core.h"

#include "usb_dcd_int.h"

extern USB_OTG_CORE_HANDLE           USB_OTG_dev;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/
// ----------------------------------------------------------------------------

uint16_t uart2_cnt = 0;
bool btransferComplete = false;

void USART2_IRQHandler( void ) {
//	++uart2_cnt;
//	USART_ClearFlag( USART2, USART_FLAG_TC );
}

void SysTick_Handler(void)
{
	schedulerTick();
}
//void DMA2_Stream0_IRQHandler(void)
//{
//	if(DMA_GetFlagStatus(DMA2_Stream0,DMA_FLAG_TCIF0) == SET)
//	{
//		btransferComplete = true;
//
//		DMA_ClearFlag(DMA2_Stream0,DMA_FLAG_TCIF0);
//		DMA_ClearITPendingBit(DMA2_Stream0,DMA_IT_TCIF0);
//
//	}
//}

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
//void HardFault_Handler(void)
//{
//  /* Go to infinite loop when Hard Fault exception occurs */
//  while (1)
//  {
//  }
//}

void hard_fault_handler_c(unsigned int * hardfault_args)
{
//	  unsigned int stacked_r0;
//	  unsigned int stacked_r1;
//	  unsigned int stacked_r2;
//	  unsigned int stacked_r3;
//	  unsigned int stacked_r12;
//	  unsigned int stacked_lr;
//	  unsigned int stacked_pc;
//	  unsigned int stacked_psr;
//
//	  stacked_r0 = ((unsigned long) hardfault_args[0]);
//	  stacked_r1 = ((unsigned long) hardfault_args[1]);
//	  stacked_r2 = ((unsigned long) hardfault_args[2]);
//	  stacked_r3 = ((unsigned long) hardfault_args[3]);
//
//	  stacked_r12 = ((unsigned long) hardfault_args[4]);
//	  stacked_lr = ((unsigned long) hardfault_args[5]);
//	  stacked_pc = ((unsigned long) hardfault_args[6]);
//	  stacked_psr = ((unsigned long) hardfault_args[7]);
//
//	  iprintf("\r\n\n Hardware fault \r\n\n");
//	  printf ("\n\n[Hard fault handler - all numbers in hex]\n");
//	  printf ("R0 = %x\n", stacked_r0);
//	  printf ("R1 = %x\n", stacked_r1);
//	  printf ("R2 = %x\n", stacked_r2);
//	  printf ("R3 = %x\n", stacked_r3);
//	  printf ("R12 = %x\n", stacked_r12);
//	  printf ("LR [R14] = %x  subroutine call return address\n", stacked_lr);
//	  printf ("PC [R15] = %x  program counter\n", stacked_pc);
//	  printf ("PSR = %x\n", stacked_psr);
//	  printf ("BFAR = %x\n", (*((volatile unsigned long *)(0xE000ED38))));
//	  printf ("CFSR = %x\n", (*((volatile unsigned long *)(0xE000ED28))));
//	  printf ("HFSR = %x\n", (*((volatile unsigned long *)(0xE000ED2C))));
//	  printf ("DFSR = %x\n", (*((volatile unsigned long *)(0xE000ED30))));
//	  printf ("AFSR = %x\n", (*((volatile unsigned long *)(0xE000ED3C))));
//	  printf ("SCB_SHCSR = %x\n", SCB->SHCSR);

	  while(1);
}
/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
//void SVC_Handler(void)
//{
//}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}


/**
  * @brief  This function handles OTG_HS Handler.
  * @param  None
  * @retval None
  */
void OTG_FS_IRQHandler(void)
{
 USBD_OTG_ISR_Handler (&USB_OTG_dev);
}


// JEK -> The following two function are handled by FreeRTOS.  See line 225 
// in port.c inside of FreeRTOS.

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
//void PendSV_Handler(void)
//{
//}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
//void SysTick_Handler(void)
//{
//}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

