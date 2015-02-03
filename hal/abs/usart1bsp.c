/*************************************************************************************
 *                                 < C/C++ SOURCE FILE >                              *
 **************************************************************************************/

/**************************************************************************************
 @file     usart1bsp.c
 @author   Usuario
 @version  v1.0.0
 @date 	  16/08/2012
 @brief

 @note
 Copyright (C) ALTRIZ - Automação Industrial


 *************************************************************************************/

/**************************************************************************************
 *                                        INCLUDES                                     *
 ***************************************************************************************/

#include <string.h>

#include "stm32f4xx_conf.h"

#include "usart1bsp.h"

#include "buffer.h"

/**************************************************************************************
 *                               DEFINITIONS AND MACROS                                *
 ***************************************************************************************/

#define USART1_DR_ADDRESS                	((uint32_t)USART1 + 0x04)

#define USART1_RX_DMA_ENABLE				1
#define USART1_TX_DMA_ENABLE				1

#define RX_FIFO_SIZE 						32
#define TX_FIFO_SIZE 						16

#define USART1_INTERRUPT_ENABLE			 	1
#define USART1_RX_VIA_INTERRUPT				0

#define TX_CIRC_BUFF_SIZE					T_BUFFER_TX_SIZE_U1
#define RX_CIRC_BUFF_SIZE 					T_BUFFER_RX_SIZE_U1

#define USART1_PREEMPTION_PRIORIRY		 	15
#define USART1_SUB_PRIORITY				 	1

#define TIM3_PREEMPTION_PRIORIRY		 	15
#define TIM3_SUB_PRIORITY				 	2

#define TX_DMA_PREEMPTION_PRIORIRY		 	15
#define TX_DMA_SUB_PRIORITY				 	3

#define RX_DMA_PREEMPTION_PRIORIRY		 	15
#define RX_DMA_SUB_PRIORITY				 	4

/**************************************************************************************
 *                          TYPEDEFS, CLASSES AND STRUCTURES                           *
 ***************************************************************************************/

/**************************************************************************************
 *                                     PROTOTYPES                                      *
 ***************************************************************************************/

static void ConfigPeripheralClock(void);
static void ConfigGPIOForUSART1(void);

#if(USART1_TX_DMA_ENABLE || USART1_RX_DMA_ENABLE)

static void ConfigDMAStreamInterruptForUSART1(void);
static void ConfigDMAForUSART1(void);

#if(USART1_RX_DMA_ENABLE)
static void ConfigTIM3ForUSART1RXDMATimeout(void);
#endif /* USART1_RX_DMA_ENABLE */

#endif /* USART1_RX_DMA_ENABLE || USART1_RX_DMA_ENABLE */

#if (USART1_INTERRUPT_ENABLE)
static void USART1InterruptConfig(void);
#endif /* USART1_INTERRUPT_ENABLE */

/**************************************************************************************
 *                                  GLOBAL VARIABLES                                   *
 ***************************************************************************************/

/**************************************************************************************
 *                                   LOCAL  VARIABLES                                  *
 ***************************************************************************************/

static uint8_t u8_rxarrayU1[RX_CIRC_BUFF_SIZE];

static uint8_t u8_txarrayU1[TX_CIRC_BUFF_SIZE];

static TBuffer T_rxcircBuffU1;

static TBuffer T_txcircBuffU1;

static uint8_t RxFIFO1[RX_FIFO_SIZE];

static uint8_t TxFIFO1[TX_FIFO_SIZE];

static bool b_usart1_initialized = false;

void (*rx1Callback)(void); //callback called when data is received
void (*tx1Callback)(void); //callback called when data is transmitted

/**************************************************************************************
 *                               FUNCTION  IMPLEMENTATION                              *
 ***************************************************************************************/

/**
 * This function init the USART1, the bauderate may be selected sent parameters for the function
 * @param[in] baudrate is a baudrate for the USART1
 */
void usart1bsp_Init(uint32_t baudrate)
{

	if(b_usart1_initialized == true)
	{
		usart1bsp_DeInit();
	}

	USART_InitTypeDef USART_InitStruct;

	/* Clock config for USART1, GPIO and DMA */
	ConfigPeripheralClock();

	/*GPIO settings for USART1*/
	ConfigGPIOForUSART1();

	USART_OverSampling8Cmd(USART1, ENABLE);

	/* Now the USART_InitStruct is used to define the
	 * properties of USART1
	 */
	USART_InitStruct.USART_BaudRate = baudrate;	// the baudrate is set to the value we passed into this init function
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	USART_InitStruct.USART_StopBits = USART_StopBits_1;	// we want 1 stop bit (standard)
	USART_InitStruct.USART_Parity = USART_Parity_No;// we don't want a parity bit (standard)
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(USART1, &USART_InitStruct); // again all the properties are passed to the USART_Init function which takes care of all the bit setting

	//This enables the complete USART1 peripheral
	USART_Cmd(USART1, ENABLE);

#if(USART1_RX_DMA_ENABLE || USART1_TX_DMA_ENABLE)
	ConfigDMAForUSART1();
	ConfigDMAStreamInterruptForUSART1();
#endif /* USART1_RX_DMA_ENABLE || USART1_TX_DMA_ENABLE */

	//Init Tbuffer to RX event
#if(USART1_RX_DMA_ENABLE)
	initBuffer(&T_rxcircBuffU1, u8_rxarrayU1, RX_CIRC_BUFF_SIZE);
#endif
#if(USART1_TX_DMA_ENABLE)
	initBuffer(&T_txcircBuffU1, u8_txarrayU1, TX_CIRC_BUFF_SIZE);
#endif

#if (USART1_INTERRUPT_ENABLE)
	USART1InterruptConfig();
#endif /* USART1_INTERRUPT_ENABLE */

	b_usart1_initialized = true;
}

/**
 * Clear all IT pendings and deinit all peripherals
 */
void usart1bsp_DeInit(void) {

	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TCIF7);
	DMA_ClearITPendingBit(DMA2_Stream5, DMA_IT_TCIF5);

	TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
	USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
	DMA_ITConfig(DMA2_Stream7, DMA_IT_TCIF7, DISABLE);
	DMA_ITConfig(DMA2_Stream5, DMA_IT_TCIF5, DISABLE);

	NVIC_ClearPendingIRQ(USART1_IRQn);
	NVIC_ClearPendingIRQ(TIM3_IRQn);
	NVIC_ClearPendingIRQ(DMA2_Stream7_IRQn);
	NVIC_ClearPendingIRQ(DMA2_Stream5_IRQn);

	NVIC_DisableIRQ(USART1_IRQn);
	NVIC_DisableIRQ(TIM3_IRQn);
	NVIC_DisableIRQ(DMA2_Stream7_IRQn);
	NVIC_DisableIRQ(DMA2_Stream5_IRQn);

	USART_DeInit(USART1);
	DMA_DeInit(DMA2_Stream7);
	DMA_DeInit(DMA2_Stream5);
	TIM_DeInit(TIM3);

//	RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART1, ENABLE);
//	RCC_AHB1PeriphResetCmd(RCC_AHB1Periph_DMA2, ENABLE);
//	RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM3, ENABLE);
}

/**
 * Enable clock for the USART1 and GPIO
 */
static void ConfigPeripheralClock(void) {
	/* enable APB2 peripheral clock for USART1
	 * note that only USART1 and USART6 are connected to APB2
	 * the other USARTs are connected to APB1
	 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	/* enable the peripheral clock for the pins used by
	 * USART1, PA9 for TX and PA10 for RX
	 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_DMA2, ENABLE);

}

/**
 *  Set PA9 and PA10 pins as RX and TX connected in USART1
 */
static void ConfigGPIOForUSART1(void) {
	GPIO_InitTypeDef GPIO_InitStruct;
	/* This sequence sets up the TX and RX pins
	 * so they work correctly with the USART1 peripheral
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // Pins 6 (TX) and 7 (RX) are used
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; // the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;	// this defines the IO speed and has nothing to do with the baudrate!
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;	// this defines the output type as push pull mode (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;// this activates the pullup resistors on the IO pins
	GPIO_Init(GPIOB, &GPIO_InitStruct);	// now all the values are passed to the GPIO_Init() function which sets the GPIO registers

	/* The RX and TX pins are now connected to their AF
	 * so that the USART1 can take over control of the
	 * pins
	 */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1 ); //
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1 );
}

#if(USART1_RX_DMA_ENABLE || USART1_TX_DMA_ENABLE)

/**
 * Configure DMA transfers streams for USART1
 */
static void ConfigDMAForUSART1(void) {
	DMA_InitTypeDef DMA_InitStructure;

#if(USART1_TX_DMA_ENABLE)

	//Enable DMA for TX transfer using the TxFIFO1
	DMA_InitStructure.DMA_PeripheralBaseAddr = USART1_DR_ADDRESS;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;

	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) TxFIFO1;
	DMA_InitStructure.DMA_BufferSize = TX_FIFO_SIZE;

	DMA_Init(DMA2_Stream7, &DMA_InitStructure);

	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);

	USART_ClearFlag(USART1, USART_FLAG_TC );
#endif /* USART1_TX_DMA_ENABLE */

#if(USART1_RX_DMA_ENABLE)

	//Enable DMA for RX transfer using the RxFIFO1
	DMA_InitStructure.DMA_PeripheralBaseAddr = USART1_DR_ADDRESS;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;

	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) RxFIFO1;
	DMA_InitStructure.DMA_BufferSize = RX_FIFO_SIZE;

	DMA_Init(DMA2_Stream5, &DMA_InitStructure);

	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);

	USART_ClearFlag(USART1, USART_FLAG_RXNE );
	USART_ClearITPendingBit(USART1, USART_IT_RXNE );
	USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

	DMA_Cmd(DMA2_Stream5, ENABLE);

	ConfigTIM3ForUSART1RXDMATimeout();

#endif /* USART1_RX_DMA_ENABLE */

}

#endif /* USART1_RX_DMA_ENABLE || USART1_TX_DMA_ENABLE */

#if (USART1_RX_DMA_ENABLE || USART1_TX_DMA_ENABLE)

/**
 * Configure the DMA interrupts.
 */
static void ConfigDMAStreamInterruptForUSART1(void) {
	NVIC_InitTypeDef NVIC_InitStructure;

#if(USART1_RX_DMA_ENABLE)
	//TX DMA
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;	// we want to configure the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =
			TX_DMA_PREEMPTION_PRIORIRY;	// this sets the priority group of the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = TX_DMA_SUB_PRIORITY;// this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	// the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);

	DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);
#endif

#if(USART1_RX_DMA_ENABLE)
	//RX DMA
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream5_IRQn;	// we want to configure the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =
			RX_DMA_PREEMPTION_PRIORIRY;	// this sets the priority group of the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = RX_DMA_SUB_PRIORITY;// this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	// the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);

	DMA_ITConfig(DMA2_Stream5, DMA_IT_TC, ENABLE);
#endif /* USART1_RX_DMA_ENABLE */

}

#endif /* USART1_RX_DMA_ENABLE || USART1_TX_DMA_ENABLE */

#if (USART1_INTERRUPT_ENABLE)

/**
 *  Case the USART1 not use DMA the data can be received through Interrupt RX handler;
 *  This function sets the interrupt for USART1.
 */
static void USART1InterruptConfig(void) {
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Here the USART1 receive interrupt is enabled
	 * and the interrupt controller is configured
	 * to jump to the USART1_IRQHandler() function
	 * if the USART1 receive interrupt occurs
	 */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn; // we want to configure the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =
			USART1_PREEMPTION_PRIORIRY;	// this sets the priority group of the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = USART1_SUB_PRIORITY;// this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	// the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);	// the properties are passed to the NVIC_Init function which takes care of the low level stuff
}

#endif /* USART1_INTERRUPT_ENABLE */

#if(USART1_RX_DMA_ENABLE)
/**
 * Configure TIM3 how timeout event for receive data in USART1
 */
static void ConfigTIM3ForUSART1RXDMATimeout(void) {
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	/* Enable the TIM3 gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =
			TIM3_PREEMPTION_PRIORIRY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = TIM3_SUB_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* TIM3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 350;  // 700 Hz down to 1 Hz
	TIM_TimeBaseStructure.TIM_Prescaler = 60000 - 1; // 144 MHz/1 Clock down to 700KHz (adjust per your clock)
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	/* TIM IT enable */
	TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
	/* TIM3 enable counter */
	TIM_Cmd(TIM3, DISABLE);
}
#endif /* USART1_RX_DMA_ENABLE */

/**
 * Sends a character.
 * @param[in] c character to put in the USART
 */
void usart1bsp_Putc(char c) {
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC ) == RESET) {
	}

	USART_SendData(USART1, (uint16_t) c);
}

/**
 * Sends a character array. Blocking the program.
 * @param[in] str string will be send to USART, is need use EOS '\0'.
 */
void usar1tbsp_PutString(const char *str) {
	while (*str != '\0') {
		usart1bsp_Putc(*str);

		str++;
	}
}

/**
 * Send buffer via USART1 without blocking the program.
 * @param buff data to be sent through USART1
 * @param u16_size size of buffer must be less that TX_FIFO_SIZE
 */
uint32_t usart1bsp_SendNBytes(uint8_t *buff, uint16_t u16_size) {

	uint32_t u32_numbytesput = 0;

	if (DMA_GetFIFOStatus(DMA2_Stream7 ) == DMA_FIFOStatus_Empty)
	{

		if (u16_size > TX_FIFO_SIZE)
		{

			uint32_t i;
			for (i = 0; i < TX_FIFO_SIZE; i++) {
				TxFIFO1[i] = *buff++;
			}

			u32_numbytesput = TX_FIFO_SIZE;

			u32_numbytesput += bufferPutN(&T_txcircBuffU1, buff, u16_size-TX_FIFO_SIZE);

			DMA_SetCurrDataCounter(DMA2_Stream7, TX_FIFO_SIZE);
			DMA_Cmd(DMA2_Stream7, ENABLE);

		} else
		{

			uint32_t i;
			for (i = 0; i < TX_FIFO_SIZE; i++) {
				TxFIFO1[i] = *buff++;
			}

			u32_numbytesput = u16_size;

			DMA_SetCurrDataCounter(DMA2_Stream7, u16_size);
			DMA_Cmd(DMA2_Stream7, ENABLE);

		}
	}
	else
	{
		NVIC_DisableIRQ(DMA2_Stream7_IRQn);
		u32_numbytesput = bufferPutN(&T_txcircBuffU1, buff, u16_size);
		NVIC_EnableIRQ(DMA2_Stream7_IRQn);
	}

	return u32_numbytesput;
}

/**
 * This function checks if the circular buffer for data receive is empty and put the data
 * in a refer buff.
 * @param[in] buff pointer to the array where the data will be stored
 * @param[in] u16_size size of buff used to store data
 * @return the number of data placed in the circular buffer.
 */
uint32_t usart1bsp_GetNBytes(uint8_t *buff, uint32_t u32_size) {
	uint32_t u32_datareceived = 0;

	uint32_t u32_bufflength = bufferGetLength(&T_rxcircBuffU1);

	if ((u32_size <= T_rxcircBuffU1.array_size) && (u32_bufflength != 0)) {
		if (u32_size <= u32_bufflength) {
			bufferGetN(&T_rxcircBuffU1, buff, u32_size);
			u32_datareceived = u32_size;
		} else {
			bufferGetN(&T_rxcircBuffU1, buff, u32_bufflength);
			u32_datareceived = u32_bufflength;
		}
	}

	return u32_datareceived;
}

void usart1bsp_ResetTxFIFO(void) {
//	memset(TxFIFO1,0,(size_t)TxFIFO1);
//	DMA_Cmd(DMA2_Stream7, DISABLE);
//	DMA_SetCurrDataCounter(DMA2_Stream7,TX_FIFO_SIZE);
}

void usart1bsp_ResetRxFIFO(void) {
//	uint8_t data = 0;
//	while(bufferGetByte(&T_rxcircBuffU1,&data)){}
}

/**
 * This function register handler to receive data. Could be used to several things
 * like ledblink, resets, as so on.
 * @param[in] callback handler executed when a data is received
 */
void usart1bsp_SetRxCallback(void(*callback)(void)){

	rx1Callback = callback;
}

/**
 * This function register handle to transmiter data. Could be used to several things
 * like ledblinks, resets, etc.
 * @param[in] callback handler executed when a data is received
 */
void usart1bsp_SetTxCallback(void(*callback)(void)){

	tx1Callback = callback;
}

/**
 * This function trigger the timer when one byte is received in USART 1.
 * This is necessary to enable the timeout case the number of data programmed
 * in RX FIFO (via DMA) not have reached. Case all data programmed is received,
 * a normal transfer complete is generated for DMA associated with USART1 RX.
 */
void USART1_IRQHandler(void) {

#if(USART1_RX_VIA_INTERRUPT)
	if((USART_GetITStatus(USART1, USART_IT_RXNE) == SET))
	{
		bufferPutByte(&T_rxcircBuffU1, (uint8_t)USART_ReceiveData(USART1));
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
#endif /* USART1_RX_VIA_INTERRUPT */

#if(USART1_RX_DMA_ENABLE)
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update );
	TIM_SetCounter(TIM3, 0);
	TIM_Cmd(TIM3, ENABLE);
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
//	NVIC_EnableIRQ(TIM3_IRQn);

	USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
	NVIC_DisableIRQ(USART1_IRQn);
#endif /* USART1_RX_DMA_ENABLE */

}

/**
 * Transfer complete handler. This interrupt is generated when USART1 TX FIFO programmed
 * is fully transfered via DMA.
 */
void DMA2_Stream7_IRQHandler(void) {

	if (DMA_GetITStatus(DMA2_Stream7, DMA_IT_TCIF7 ) == SET) {

		if(tx1Callback != NULL)
		{
			tx1Callback();
		}

		DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7 );
		DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TCIF7 );

		//Always is transfered  TX_FIFO_SIZE bytes, if if the buffer has less elements than TX_FIFO_SIZE
		//only available bytes are transfered
		uint32_t u32_lenght = bufferGetN(&T_txcircBuffU1, TxFIFO1,
				TX_FIFO_SIZE);

		if (u32_lenght != 0) {

			DMA_SetCurrDataCounter(DMA2_Stream7, (uint16_t) u32_lenght);
			DMA_Cmd(DMA2_Stream7, ENABLE);

		}
	}
}

/**
 * Transfer complete handler for USART1 RX. Interrupt occurs when receive the number of data programmed
 * in RX_FIFO_SIZE. The data receive is placed in circular buffer using bufferPutN, the timer is disabled
 * and the USART RXe interrupt is enabled
 */
void DMA2_Stream5_IRQHandler(void) {
	USART_DMACmd(USART1, USART_DMAReq_Rx, DISABLE);

	if (DMA_GetITStatus(DMA2_Stream5, DMA_IT_TCIF5 ) == SET) {

		if(rx1Callback != NULL)
		{
			rx1Callback();
		}

		TIM_Cmd(TIM3, DISABLE);
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update );

		bufferPutN(&T_rxcircBuffU1, RxFIFO1, RX_FIFO_SIZE);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		NVIC_ClearPendingIRQ(USART1_IRQn);
		NVIC_EnableIRQ(USART1_IRQn);

		DMA_ClearFlag(DMA2_Stream5, DMA_FLAG_TCIF5 );
		DMA_ClearITPendingBit(DMA2_Stream5, DMA_IT_TCIF5 );
	}

	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
	DMA_Cmd(DMA2_Stream5, ENABLE);

}

/**
 * If a transfer complete not is generated by the DMA responsible for managing the Receive data in
 * USART1 a timeout is generated for TIMER and the number of data received is transfer for the circular
 * buffer.
 */
void TIM3_IRQHandler(void) {
	static volatile uint16_t u16_numberOfDataTransfered = 0;

	if (TIM_GetITStatus(TIM3, TIM_IT_Update )) {

		if(rx1Callback != NULL)
		{
			rx1Callback();
		}

		TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);

		//Get current number of bytes in DMA FIFO
		u16_numberOfDataTransfered = (RX_FIFO_SIZE
				- DMA_GetCurrDataCounter(DMA2_Stream5 ));

		//Disable DMA to get bytes on FIFO
		DMA_Cmd(DMA2_Stream5, DISABLE);

		//Reset DMA data pool and clear pending interrupt
		DMA_SetCurrDataCounter(DMA2_Stream5, RX_FIFO_SIZE);
		DMA_ClearITPendingBit(DMA2_Stream5, DMA_IT_TCIF5 );
		NVIC_ClearPendingIRQ(DMA2_Stream5_IRQn);

		//put to circular buff
		bufferPutN(&T_rxcircBuffU1, RxFIFO1, u16_numberOfDataTransfered);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		NVIC_ClearPendingIRQ(USART1_IRQn);
		NVIC_EnableIRQ(USART1_IRQn);

		DMA_Cmd(DMA2_Stream5, ENABLE);

		TIM_ClearITPendingBit(TIM3, TIM_IT_Update );

	}
}

/******************************* (C) COPYRIGHT ALTRIZ *********************************/

/*******************************      END OF FILE     *********************************/
