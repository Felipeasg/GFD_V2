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


#ifndef DDS_H_
#define DDS_H_

/* Includes -----------------------------------------------------------------*/

#include <stdint.h>

/* Private macro ------------------------------------------------------------*/

#define DDS_SAMPLE_FREQ		1000000
#define DDS_ACC_MAX_VALUE	4294967295UL

/* Private function prototypes ----------------------------------------------*/

void ddsInit(uint32_t f0, float gain);
void ddsStop();

#endif /* DDS_H_ */
