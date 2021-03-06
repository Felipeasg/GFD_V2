/*
 * GFDMonitor.c
 *
 *  Created on: 03/07/2013
 *      Author: Computador
 */

#include <stdbool.h>
#include <stdint.h>

#include "GFDMonitor.h"
#include "GFDProtocol.h"
#include "stm32f4_discovery.h"
#include "dds.h"
#include "scheduler.h"
#include "meassurements.h"

uint8_t faultDetectEnable = 0x00;
uint8_t relayState = 0x00;
float dacGain = 1;
uint32_t frequency = 10000;
uint8_t generateSignalEnabled = 0;
uint8_t faultMask = 0x00;

#define FAULT_DETECT_ON_POSITIVE_BUS		STM_EVAL_GFDEDSensorGetState(SENSOR_ED1)
#define FAULT_DETECT_ON_NEGATIVE_BUS		STM_EVAL_GFDEDSensorGetState(SENSOR_ED2)

#define SET_LED_NEGATIVE_BUS_IN_FAULT 		STM_EVAL_LEDOn(LED4)
#define CLEAR_LED_NEGATIVE_BUS_IN_FAULT		STM_EVAL_LEDOff(LED4)

#define SET_LED_POSITIVE_BUS_IN_FAULT		STM_EVAL_LEDOn(LED5)
#define CLEAR_LED_POSITIVE_BUS_IN_FAULT		STM_EVAL_LEDOff(LED5)

#define CLOSE_POSITIVE_BUS_RELAY			STM_EVAL_GFDRelayOn(RELAY_1BP); \
		STM_EVAL_GFDRelayOn(RELAY_2BP); \
		STM_EVAL_LEDOn(LED3);

#define OPEN_POSITIVE_BUS_RELAY				STM_EVAL_GFDRelayOff(RELAY_1BP); \
		STM_EVAL_GFDRelayOff(RELAY_2BP); \
		STM_EVAL_LEDOff(LED3);

#define CLOSE_NEGATIVE_BUS_RELAY			STM_EVAL_GFDRelayOn(RELAY_1BN); \
		STM_EVAL_GFDRelayOn(RELAY_2BN); \
		STM_EVAL_LEDOn(LED6);

#define OPEN_NEGATIVE_BUS_RELAY				STM_EVAL_GFDRelayOff(RELAY_1BN); \
		STM_EVAL_GFDRelayOff(RELAY_2BN); \
		STM_EVAL_LEDOff(LED6);

TTaskHandler *GFDMonitorTaskHandler;

static void GFDMonitorTask(void);
static uint8_t checkBusFaultOnOff(void);
static uint8_t checkBusFaultSensor(void);

void GFDMonitor_Init(void)
{
	STM_EVAL_LEDInit(LED3);
	STM_EVAL_LEDInit(LED4);
	STM_EVAL_LEDInit(LED5);
	STM_EVAL_LEDInit(LED6);

	STM_EVAL_GFDRelayInit(RELAY_1BP);
	STM_EVAL_GFDRelayInit(RELAY_2BP);
	STM_EVAL_GFDRelayInit(RELAY_1BN);
	STM_EVAL_GFDRelayInit(RELAY_2BN);

	STM_EVAL_GFDRelayOff(RELAY_1BP);
	STM_EVAL_GFDRelayOff(RELAY_2BP);
	STM_EVAL_GFDRelayOff(RELAY_1BN);
	STM_EVAL_GFDRelayOff(RELAY_2BN);

	//	STM_EVAL_GFDEDSensorInit(SENSOR_ED1, SENSOR_MODE_GPIO);
	//	STM_EVAL_GFDEDSensorInit(SENSOR_ED2, SENSOR_MODE_GPIO);

	if(GFDMonitorTaskHandler != NULL)
		schedulerDelete(&GFDMonitorTaskHandler);

	schedulerSchedule(&GFDMonitorTaskHandler, 100, &GFDMonitorTask, true);
}


static void GFDMonitorTask(void)
{
	static uint32_t timeLedBlinkCount = 0;
	static uint32_t timeToCloseRelayNEG = 0;
	static uint32_t timeToCloseRelayPOS = 0;
	static bool flagRelayEn = false;

	uint8_t u8_temp = 0;
	uint32_t u32_temp = 0;
	float f_temp = 0.0;

	if(GFDProtocol_GetData(MESSAGE_ENABLE_FAULT_DETECT, &u8_temp, sizeof(uint8_t)) == true)
	{
		faultDetectEnable = u8_temp;
		GFDProtocol_SendCommandExecutedPakage(MESSAGE_ENABLE_FAULT_DETECT);
	}

	if(faultDetectEnable != 0x00)
	{
#if 1
		//		faultMask = checkBusFaultOnOff();
		faultMask = checkBusFaultSensor();
#else
		if(FAULT_DETECT_ON_POSITIVE_BUS)
		{
			SET_LED_POSITIVE_BUS_IN_FAULT;
			faultMask |= (1 << 0);
		}
		else
		{
			CLEAR_LED_POSITIVE_BUS_IN_FAULT;
			faultMask &=~ (1 << 0);
		}

		if(FAULT_DETECT_ON_NEGATIVE_BUS)
		{
			SET_LED_NEGATIVE_BUS_IN_FAULT;
			faultMask |= (1 << 1);
		}
		else
		{
			CLEAR_LED_NEGATIVE_BUS_IN_FAULT;
			faultMask &=~ (1 << 1);
		}

		if((!FAULT_DETECT_ON_NEGATIVE_BUS) && (!FAULT_DETECT_ON_POSITIVE_BUS))
		{
			static bool flag = false;

			faultMask = 0;

			if(timeLedBlinkCount++ > 10)
			{
				if(flag == false)
				{
					SET_LED_POSITIVE_BUS_IN_FAULT;
					SET_LED_NEGATIVE_BUS_IN_FAULT;
					flag = true;
				}
				else
				{
					CLEAR_LED_NEGATIVE_BUS_IN_FAULT;
					CLEAR_LED_POSITIVE_BUS_IN_FAULT;
					flag = false;
				}
			}
		}
#endif
	}
	else
	{
		SET_LED_NEGATIVE_BUS_IN_FAULT;
		SET_LED_POSITIVE_BUS_IN_FAULT;
	}

	if(GFDProtocol_GetData(MESSAGE_RELAY_STATE, &u8_temp, sizeof(uint8_t)) == true)
	{
		relayState = u8_temp;
		flagRelayEn = true;
		timeToCloseRelayPOS = 0;
		timeToCloseRelayNEG = 0;
		GFDProtocol_SendCommandExecutedPakage(MESSAGE_RELAY_STATE);
	}



	if(relayState == 0x00 && flagRelayEn == true)
	{
		OPEN_NEGATIVE_BUS_RELAY;
		OPEN_POSITIVE_BUS_RELAY;

		timeToCloseRelayPOS = 0;
		timeToCloseRelayNEG = 0;

		flagRelayEn = false;
	}
	else if(relayState == 0x01 && flagRelayEn == true)
	{
		OPEN_NEGATIVE_BUS_RELAY;

		if(timeToCloseRelayPOS++ > 5)
		{
			CLOSE_POSITIVE_BUS_RELAY;
			timeToCloseRelayPOS = 0;
			flagRelayEn = false;
		}


	}
	else if(relayState == 0x02 && flagRelayEn == true)
	{
		OPEN_POSITIVE_BUS_RELAY;

		if(timeToCloseRelayNEG++ > 5)
		{
			CLOSE_NEGATIVE_BUS_RELAY;
			timeToCloseRelayNEG = 0;
			flagRelayEn = false;
		}


	}


	if(GFDProtocol_GetData(MESSAGE_ENABLE_GENERATE_SIGNAL, &u8_temp, sizeof(uint8_t)) == true)
	{
		generateSignalEnabled = u8_temp;
		GFDProtocol_SendCommandExecutedPakage(MESSAGE_ENABLE_GENERATE_SIGNAL);

		if(generateSignalEnabled != 0)
		{
			//			generateSignalEnabled = 0;
			ddsInit(frequency, dacGain);
		}
		else
		{
			ddsInit(1000,0);
			ddsStop();
		}
	}
	if(GFDProtocol_GetData(MESSAGE_SET_FREQ, &u32_temp, sizeof(uint32_t)) == true)
	{
		frequency = u32_temp;
		GFDProtocol_SendCommandExecutedPakage(MESSAGE_SET_FREQ);
	}

	if(GFDProtocol_GetData(MESSAGE_SET_GAIN, &f_temp, sizeof(float)) == true)
	{
		dacGain = f_temp;
		GFDProtocol_SendCommandExecutedPakage(MESSAGE_SET_GAIN);
	}

	if(GFDProtocol_GetData(MESSAGE_GET_CURRENT, NULL, 0) == true)
	{
		float current = 0.0;

		current = meassurements_GetCurrRMS();

		GFDProtocol_SendData(MESSAGE_GET_CURRENT, &current, sizeof(float));
	}

	if(GFDProtocol_GetData(MESSAGE_GET_FAULT_MASK, NULL, 0) == true)
	{
		GFDProtocol_SendData(MESSAGE_GET_FAULT_MASK, &faultMask, sizeof(uint8_t));
	}

}

static uint8_t checkBusFaultOnOff(void)
{
	static uint32_t timeLedBlinkCount = 0;
	uint8_t u8_fault = 0;

	if(FAULT_DETECT_ON_POSITIVE_BUS)
	{
		SET_LED_POSITIVE_BUS_IN_FAULT;
		u8_fault |= (1 << 0);
	}
	else
	{
		CLEAR_LED_POSITIVE_BUS_IN_FAULT;
		u8_fault &=~ (1 << 0);
	}

	if(FAULT_DETECT_ON_NEGATIVE_BUS)
	{
		SET_LED_NEGATIVE_BUS_IN_FAULT;
		u8_fault |= (1 << 1);
	}
	else
	{
		CLEAR_LED_NEGATIVE_BUS_IN_FAULT;
		u8_fault &=~ (1 << 1);
	}

	if((!FAULT_DETECT_ON_NEGATIVE_BUS) && (!FAULT_DETECT_ON_POSITIVE_BUS))
	{
		static bool flag = false;

		u8_fault = 0;

		if(timeLedBlinkCount++ > 10)
		{
			if(flag == false)
			{
				SET_LED_POSITIVE_BUS_IN_FAULT;
				SET_LED_NEGATIVE_BUS_IN_FAULT;
				flag = true;
			}
			else
			{
				CLEAR_LED_NEGATIVE_BUS_IN_FAULT;
				CLEAR_LED_POSITIVE_BUS_IN_FAULT;
				flag = false;
			}
		}
	}


	return u8_fault;
}

static uint8_t checkBusFaultSensor(void)
{
	uint8_t u8_fault;
	static uint32_t timeLedBlinkCount = 0;
	float f_vT = 0.0;
	float f_vP = 0.0;
	float f_vN = 0.0;

	f_vT = meassurements_GetVT(); //obtem a tensao vT em porcentagem
	f_vT = f_vT/1.2652014;

	f_vP = meassurements_GetVP();
	f_vP = f_vP/1.267619;

	if(f_vP >= MIN_TO_CONSIDERERE_BUS_ACTIVE)
	{
		if(f_vT >= (0.5 + HYSTERESIS_TO_DETECT_FAULT))
		{
			SET_LED_NEGATIVE_BUS_IN_FAULT;
			u8_fault |= (1 << 1);
		}
		else
		{
			CLEAR_LED_NEGATIVE_BUS_IN_FAULT;
			u8_fault &=~ (1 << 1);
		}

		if(f_vT <= (0.5 - HYSTERESIS_TO_DETECT_FAULT))
		{
			SET_LED_POSITIVE_BUS_IN_FAULT;
			u8_fault |= (1 << 0);
		}
		else
		{
			CLEAR_LED_POSITIVE_BUS_IN_FAULT;
			u8_fault &=~ (1 << 0);
		}

		if((f_vT > (0.5 + HYSTERESIS_TO_DETECT_FAULT))
				&& (f_vT < (0.5 - HYSTERESIS_TO_DETECT_FAULT)))
		{
			SET_LED_POSITIVE_BUS_IN_FAULT;
			SET_LED_NEGATIVE_BUS_IN_FAULT;
		}
	}
	else
	{
		static bool flag = false;

		u8_fault = 0;

		if(timeLedBlinkCount++ > 10)
		{
			if(flag == false)
			{
				SET_LED_POSITIVE_BUS_IN_FAULT;
				SET_LED_NEGATIVE_BUS_IN_FAULT;
				flag = true;
			}
			else
			{
				CLEAR_LED_NEGATIVE_BUS_IN_FAULT;
				CLEAR_LED_POSITIVE_BUS_IN_FAULT;
				flag = false;
			}
		}
	}
	return u8_fault;

}
