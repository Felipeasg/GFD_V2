/**
 ******************************************************************************
 * @file
 * @author
 * @version
 * @date
 * @brief
 *
 *
 ******************************************************************************
 * @attention
 *
 *
 * <h2><center>&copy; COPYRIGHT 2011</center></h2>
 ******************************************************************************
 */

/* Includes -----------------------------------------------------------------*/

#include "dac_bsp.h"

#include "stm32f4xx_conf.h"

/* Private define ------------------------------------------------------------*/

#define DAC_DHR12R1_ADDRESS    0x40007408
#define DAC_DHR8R1_ADDRESS     0x40007410
#define DAC_DHR12R2_ADDRESS    0x40007414

/* Private variables --------------------------------------------------------*/

DAC_InitTypeDef DAC_InitStructure;

const uint16_t Sine12bit[32] = { 2047, 2447, 2831, 3185, 3498, 3750, 3939, 4056,
		4095, 4056, 3939, 3750, 3495, 3185, 2831, 2447, 2047, 1647, 1263, 909,
		599, 344, 155, 38, 0, 38, 155, 344, 599, 909, 1263, 1647 };

uint32_t wave_length;
uint16_t wave_table[4096];

/* Private function prototypes ----------------------------------------------*/

static void dacClockConfig(void);
static void dacGpioConfig(void);
/**
 * @brief  Load wave table values
 * @param  None
 * @retval None
 */
void dacSetWaveTableLength(int32_t length) {

	wave_length = length;

}

/**
 * @brief  Load wave table values
 * @param  None
 * @retval None
 */
void dacLoadWaveTableValueAt(uint16_t data, uint32_t index, float gain) {

	wave_table[index] = (uint16_t)((float)data*gain);

}

/**
 * @brief  Load wave table values
 * @param  None
 * @retval None
 */
void dacLoadWaveTable(uint16_t *data, uint32_t length) {

	int32_t i;

	for(i=0; i< length; i++) {

		wave_table[i] = *data++;

	}

	wave_length = length;

	DAC_Ch1_SineWaveConfig();
}

/**
  * @brief  TIM6 Configuration
  * @note   TIM6 configuration is based on CPU @168MHz and APB1 @42MHz
  * @note   TIM6 Update event occurs each 37.5MHz/256 = 16.406 KHz
  * @param  None
  * @retval None
  */
void TIM6_Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	/* TIM6 Periph clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

	/* Time base configuration */
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = 83;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);

	/* TIM6 TRGO selection */
	TIM_SelectOutputTrigger(TIM6, TIM_TRGOSource_Update);

	/* TIM6 enable counter */
	TIM_Cmd(TIM6, ENABLE);
}

/**
 * @brief  DAC  Channel1 SineWave Configuration
 * @param  None
 * @retval None
 */
void dacStart(void) {

	TIM6_Config();

	dacClockConfig();
	dacGpioConfig();

	DMA_InitTypeDef DMA_InitStructure;

	/* DAC channel1 Configuration */
	DAC_InitStructure.DAC_Trigger = DAC_Trigger_T6_TRGO;
	DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
	DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
	DAC_Init(DAC_Channel_1, &DAC_InitStructure);

	/* DMA1_Stream5 channel7 configuration **************************************/
	DMA_DeInit(DMA1_Stream5);
	DMA_InitStructure.DMA_Channel = DMA_Channel_7;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) DAC_DHR12R1_ADDRESS;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) &wave_table;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_BufferSize = wave_length;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream5, &DMA_InitStructure);

	/* Enable DMA1_Stream5 */
	DMA_Cmd(DMA1_Stream5, ENABLE);

	/* Enable DAC Channel1*/
	DAC_Cmd(DAC_Channel_1, ENABLE);

	/* Enable DMA for DAC Channel1 */
	DAC_DMACmd(DAC_Channel_1, ENABLE);
}


/**
 * @brief  DAC  Channel1 SineWave Configuration
 * @param  None
 * @retval None
 */
void DAC_Ch1_SineWaveConfig(void) {

	DMA_InitTypeDef DMA_InitStructure;

	/* DAC channel1 Configuration */
	DAC_InitStructure.DAC_Trigger = DAC_Trigger_T6_TRGO;
	DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
	DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
	DAC_Init(DAC_Channel_1, &DAC_InitStructure);

	/* DMA1_Stream5 channel7 configuration **************************************/
	DMA_DeInit(DMA1_Stream5);
	DMA_InitStructure.DMA_Channel = DMA_Channel_7;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) DAC_DHR12R1_ADDRESS;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) &wave_table;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_BufferSize = wave_length;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream5, &DMA_InitStructure);

	/* Enable DMA1_Stream5 */
	DMA_Cmd(DMA1_Stream5, ENABLE);

	/* Enable DAC Channel1*/
	DAC_Cmd(DAC_Channel_1, ENABLE);

	/* Enable DMA for DAC Channel1 */
	DAC_DMACmd(DAC_Channel_1, ENABLE);
}

void dacStop() {

	DAC_DeInit();

}

static void dacClockConfig(void)
{
	/* DMA1, DMA2 clock. GPIOA clock enable (to be used with DAC) and  GPIOC*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1 | RCC_AHB1Periph_GPIOA, ENABLE);

	/* DAC Periph clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

}

static void dacGpioConfig(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* DAC channel 1 (DAC_OUT1 = PA.4) configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}
