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

#ifndef DAC_BSP_H_
#define DAC_BSP_H_

/* Includes -----------------------------------------------------------------*/

#include <stdint.h>

/* Private macro ------------------------------------------------------------*/

/* Private function prototypes ----------------------------------------------*/

/**
 * @brief  Load wave table values
 * @param  None
 * @retval None
 */
void dacLoadWaveTable(uint16_t *data, uint32_t length);

/**
 * @brief  DAC  Channel2 SineWave Configuration
 * @param  None
 * @retval None
 */
void DAC_Ch1_SineWaveConfig(void);

void dacStart(void);
void dacStop();
void dacSetWaveTableLength(int32_t length);
void dacLoadWaveTableValueAt(uint16_t data, uint32_t index, float gain);

#endif /* DAC_BSP_H_ */
