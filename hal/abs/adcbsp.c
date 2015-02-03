/**************************************************************************************
 *                                 < C/C++ SOURCE FILE >                              *
 **************************************************************************************/

/**
 **************************************************************************************
    @file     adcbsp.c                                                            
    @author   Usuario	                                                              
    @version  v1.0.0                                                                  
    @date 	  10/08/2012                                                                 
    @brief                                                                            

    @note
		 Copyright (C) ALTRIZ - Automa��o Industrial                                                            

 **************************************************************************************
 */

/***************************************************************************************
 *                                        INCLUDES                                     *
 ***************************************************************************************/


#include "adcbsp.h"
#include "stm32f4xx.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_it.h"

/***************************************************************************************
 *                               DEFINITIONS AND MACROS                                *
 ***************************************************************************************/

#define ADC_CDR_ADDRESS    							((uint32_t)0x40012308)
//#define ADC_CDR_ADDRESS	(ADC1_BASE + 0x4C)

#define SYSTEM_CORE_CLOCK_IN_HZ 					SystemCoreClock //!< System clock in Hertz
#define TIM2_COUNTER_CLOCK_IN_HZ					600000 	  		//!< TIMER 2 clock in Hertz
#define TIM2_TRIGGER_IN_HZ							10000       		//!< Trigger for ADC conversion in Hertz

#define NUMBER_OF_RANKS_FOR_SCAN_MODE				1
#define ADC_TRIPLE_MODE								3
#define NUMBER_OF_SCAN_READINGS						1 				/*!< One reading include all channels programmed.
											   	   	       	   	   	   	 For instance, with the ADC triple mode and two scan
											   	   	   	   	   	   	   	 readings the buffer size will have 12 positions
											   	   	   	   	   	   	   	 because each reading has 6 samples. This
											   	   	   	   	   	   	   	 also determine the DMA interruption Time.*/
#if 1
#define DMA_BUFFER_SIZE								((NUMBER_OF_SCAN_READINGS)* \
                                        			(ADC_TRIPLE_MODE*NUMBER_OF_RANKS_FOR_SCAN_MODE)) /*!< Size of buffer transfered by the DMA. It should
                                          	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  a plenty of number of ADC conversions where each element
                                          	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  has is a one word size, for ADC triple mode simultaneous*/
#endif

//#define DMA_BUFFER_SIZE								NUMBER_OF_SAMPLES

#define DMA_ADC_TRANSF_PREEMPTION_PRIORIRY 		    1
#define DMA_ADC_TRANSF_SUBPRIORITY					1

/***************************************************************************************
 *                          TYPEDEFS, CLASSES AND STRUCTURES                           *
 ***************************************************************************************/

/***************************************************************************************
 *                                     PROTOTYPES                                      *
 ***************************************************************************************/

static void prvSetTriggerForADC(void);
static void prvRCCConfig(void);
static void prvGPIOConfig(void);
static void prvSetDMATransfer(void);

/**************************************************************************************
 *                                  GLOBAL VARIABLES                                   *
 ***************************************************************************************/

__IO uint32_t ADCTripleConvertedValue[DMA_BUFFER_SIZE]; //!< Store the ADC conversions. Data is transferred via DMA.

volatile bool done = false;
static bool b_adcInitialized = false;

/**************************************************************************************
 *                                   LOCAL  VARIABLES                                  *
 ***************************************************************************************/

static Tf_ADCSamplesDoneHandleRequest *pfSamplesDoneReqHandler = NULL;

/**************************************************************************************
 *                               FUNCTION  IMPLEMENTATION                              *
 ***************************************************************************************/

/**
 * This function configure and enable the timer 2 as a trigger for ADC.
 */
static void prvSetTriggerForADC(void)
{

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef         TIM_OCInitStructure;

	uint16_t u16_prescalerValue = 0, u16_CCR4_val = 0;

	/* Timer trigger configuration*/
	/* Update_Timer_rate = TIM2_COUNTER_CLOCK/CCR4 */
	/*FIXME: The timer frequency is not determined only the capture channel. The timer frequency is determinated
	 * by the TIM_Period, this justifies the changes in this function.*/
	u16_CCR4_val = (uint16_t)(TIM2_COUNTER_CLOCK_IN_HZ/TIM2_TRIGGER_IN_HZ);

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = u16_CCR4_val;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	/* Compute the prescaler value based in SYSTEM CLOCK and TIMER CLOCK*/
	u16_prescalerValue = (uint16_t) (((SYSTEM_CORE_CLOCK_IN_HZ / 4) / TIM2_COUNTER_CLOCK_IN_HZ) - 1);

	TIM_PrescalerConfig(TIM2, u16_prescalerValue, TIM_PSCReloadMode_Immediate);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Toggle; //OCxRef toggles when the counter matches capture/compare register
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //enable trigger output (TRGO)
	TIM_OCInitStructure.TIM_Pulse = u16_CCR4_val;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);

	TIM_SelectOutputTrigger(TIM2,TIM_TRGOSource_OC4Ref);
	TIM_SelectMasterSlaveMode(TIM2,TIM_MasterSlaveMode_Enable);

}

/**
 * This function enable the peripheral clock involved in ADC conversion
 */
static void prvRCCConfig( void )
{
	/* Enable peripheral clocks */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOA, ENABLE);// | RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 | RCC_APB2Periph_ADC3, ENABLE); //| RCC_APB2Periph_ADC2 | RCC_APB2Periph_ADC3 , ENABLE);

}

/**
 * Configure GPIO ports PA1, PA2, PA3 and PB1 as analog input
 */
static void prvGPIOConfig( void )
{
	GPIO_InitTypeDef      GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/**
 * Configure DMA transfer for ADC peripheral and enable DMA2_stream_0 interrupt to warn that data is transferred
 */
static void prvSetDMATransfer( void )
{
	DMA_InitTypeDef       DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* DMA2 Stream0 channel0 configuration */
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC_CDR_ADDRESS;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADCTripleConvertedValue;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = DMA_BUFFER_SIZE; //FIXME: Must be a multiple of the number of channels
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);

	/* DMA2_Stream0 enable */
	DMA_Cmd(DMA2_Stream0, ENABLE);

	/* Enable the DMA2 Stream0 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = DMA_ADC_TRANSF_PREEMPTION_PRIORIRY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = DMA_ADC_TRANSF_SUBPRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	DMA_ITConfig(DMA2_Stream0,DMA_IT_TC,ENABLE);
}

/**
 * Initialize ADC in Triple and Scan simultaneous mode with transfer via DMA.
 *
 */
void adcbsp_Init( void )
{
	ADC_InitTypeDef       ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;


	if(b_adcInitialized == true)
	{
		adcbsp_DeInit();

		b_adcInitialized = false;
	}

	/******************************************************************************/
	/*     ADCs interface clock, trigger, pins and DMA configuration              */
	/******************************************************************************/

	/*Clock for all peripheral involved in ADC conversions*/
	prvRCCConfig();

	/*Enable analog inputs*/
	prvGPIOConfig();

	/*TIMER*/
	prvSetTriggerForADC();

	/*Configure DMA*/
	prvSetDMATransfer();

	/******************************************************************************/
	/*  ADCs configuration: triple mode simultaneous in Scan mode                 */
	/******************************************************************************/

	/* After the timer 2 trigger the three ADCs are converted as follows:
	 *
	 *           R
	 *           A
	 *           N
	 *           K
	 *          (1)
	 *         -------------
 	 *  ADC1   |CH1|    ...    PA1 - AD_VP
 	 *         -------------
 	 *         -------------
 	 *  ADC2   |CH2|    ...	   PA2 - AD_VT
 	 *         -------------
 	 *         -------------
 	 *  ADC3   |CH3|    ...    PA3 - AD_COR
 	 *         -------------
 	 *         .		 .
 	 *        / \       / \
 	 *        | |       | |
 	 *         -         -
 	 *     TIMER 2      DMA
 	 *     Trigger    Transfer
 	 *
 	 * After the conversions the data is transfered to memory via DMA.
 	 * The DMA transfer is only possible by ADC regular conversion
	 */

	/* ADC Common configuration *************************************************/
	ADC_CommonInitStructure.ADC_Mode = ADC_TripleMode_RegSimult;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInit(&ADC_CommonInitStructure);

	/* ADC1 regular channel configuration ************************************/
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE; //??
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; //Inject channels cannot be converted continuously.
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO; //Regular channel trigger
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 1; //Number of SCAN conversions or number of RANK conversions

	ADC_Init(ADC1, &ADC_InitStructure);
	/* ADC1 RANK configuration */
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_3Cycles); //RANK 1 - ADC1 read channel 1


	/* Enable ADC1 DMA */
	/* ADC1 is mater, is not necessary enable DMA for ADC2 and ADC3*/
	ADC_DMACmd(ADC1, ENABLE);


	/* ADC2 regular channel configuration ************************************/
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE; //SCAN all channels enable in RANK
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; //ADC conversion triggered for timer
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO; //Regular channel trigger
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 1; //Number of SCAN conversions or number of RANK conversions

	ADC_Init(ADC2, &ADC_InitStructure);
	/* ADC2 RANK configuration */
	ADC_RegularChannelConfig(ADC2, ADC_Channel_2, 1, ADC_SampleTime_3Cycles); //RANK 1 - ADC2 read channel 2


	/* ADC3 regular channel configuration ************************************/
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; //Inject channels cannot be converted continuously.
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO; //Regular channel trigger
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 1; //Number of SCAN conversions or number of RANK conversions

	/* ADC3 regular channel 2 configuration ************************************/
	ADC_Init(ADC3, &ADC_InitStructure);
	/* ADC3 RANK configuration */
	ADC_RegularChannelConfig(ADC3, ADC_Channel_3, 1, ADC_SampleTime_3Cycles); //RANK 1 - ADC3 read channel 3


	/* Enable DMA request after last transfer (multi-ADC mode) ******************/
	ADC_MultiModeDMARequestAfterLastTransferCmd(ENABLE);

	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);

	/* Enable ADC2 */
	ADC_Cmd(ADC2, ENABLE);

	/* Enable ADC3 */
	ADC_Cmd(ADC3, ENABLE);

	b_adcInitialized = true;

}

/**
 * Start the timer (TIM2) to trigger ADCs
 *
 */
void adcbsp_StartSamples(void)
{
	/* TIM1 counter enable to trigger ADC*/
	TIM_Cmd(TIM2, ENABLE);
}

/**
 * Stop the timer that trigger ADCs
 */
void adcbsp_StopSamples(void)
{
	/* TIM1 counter enable to trigger ADC*/
	TIM_Cmd(TIM2, DISABLE);
}

/**
 * Deinit all peripherals and disables IT pending
 */
void adcbsp_DeInit(void)
{
	DMA_ITConfig(DMA2_Stream0,DMA_IT_TC,DISABLE);
	NVIC_ClearPendingIRQ(DMA2_Stream0_IRQn);

	ADC_DeInit();

	TIM_DeInit(TIM2);

	DMA_DeInit(DMA2_Stream0);
}

/**
 * This function Register a callback to the transfer complete DMA event. this function
 * passes the samples to another modules.
 * @param[in] pfHandler pointer to function that is called in the DMA event.
 */
void adcbsp_RegisterADCHandler(Tf_ADCSamplesDoneHandleRequest pfHandler)
{
	if(pfHandler != NULL)
	{
		pfSamplesDoneReqHandler = pfHandler;

//		TIM_Cmd(TIM2, ENABLE);
	}
}

/**
 * ADC transfer complete interrupt. This interrupt occurs always that all samples of the ADC
 * is transfered to the ADCTripleConvertedValue via DMA. Here the callback is called and the
 * samples is transfered.
 */
void DMA2_Stream0_IRQHandler(void)
{
	if(DMA_GetFlagStatus(DMA2_Stream0,DMA_FLAG_TCIF0) == SET)
	{

		if(pfSamplesDoneReqHandler != NULL)
		{
			pfSamplesDoneReqHandler(ADCTripleConvertedValue);
		}

		DMA_ClearITPendingBit(DMA2_Stream0,DMA_IT_TCIF0);
		DMA_ClearFlag(DMA2_Stream0,DMA_FLAG_TCIF0);
	}
}
/******************************* (C) COPYRIGHT ALTRIZ *********************************/

/*******************************      END OF FILE     *********************************/
