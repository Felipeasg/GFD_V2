/*
 * GFDProtocol.c
 *
 *  Created on: 03/07/2013
 *      Author: Computador
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "GFDProtocol.h"
#include "scheduler.h"
#include "buffer.h"
#include "stream.h"
#include "crc16.h"

#define GFD_PREAMBULE_SIZE 2
#define GFD_FUNCTION_SIZE 1
#define GFD_PREAMBLE_HIGH  0xAA
#define GFD_PREAMBLE_LOW   0x55


//source | dest | checksum
#define HEADER_BUFF_SIZE 			4

//Defines para os campos do cabeçalho no headerbuff
#define HEADER_FIELD_SOURCE	 		0
#define HEADER_FIELD_DEST 			1
#define HEADER_FIELD_SIZE			2
#define HEADER_FIELD_CHECKSUM		3


#define GFD_ID						0x34

uint8_t u8_headerBuff[HEADER_BUFF_SIZE];

TTaskHandler *gfdProtocolTaskHandler;

TBuffer codecReadBuffer;
uint8_t codecReadBufferArray[64];

TBuffer codecWriteBuffer;
uint8_t codecWriteBufferArray[64];

static uint8_t mu8_messageType = 0;
static uint8_t mu8_messageSize = 0;
static uint8_t mu8_messageBuff[255];

typedef enum {
	SYNC = 0,
	GET_HEADER,
	GET_DATA,
	CRC
}EN_GFDProtocolTaskState;

static uint8_t calcCheckSum(void *data, uint8_t u8_size);

bool GFDProtocol_GetData(uint8_t messageType, void *data, uint8_t u8_size)
{
	bool b_result = false;
	if(data != NULL && u8_size != 0)
	{
		if(messageType == mu8_messageType)
		{
			if(u8_size == mu8_messageSize)
			{
				memcpy(data,mu8_messageBuff,mu8_messageSize);

				mu8_messageSize = 0;
				mu8_messageType = 0;
				memset(mu8_messageBuff,0,sizeof(mu8_messageBuff));
				b_result = true;

			}
		}
	}
	else
	{
		if(messageType == mu8_messageType)
		{
			mu8_messageType = 0;
			b_result = true;
		}
	}

	return b_result;
}

void GFDProtocol_SendData(uint8_t u8_function, void *data, uint8_t u8_size)
{
	uint8_t bufferTemp[10] = {0};
	uint8_t u8_headerBuffToSend[HEADER_BUFF_SIZE];

	u8_headerBuffToSend[HEADER_FIELD_SOURCE] = GFD_ID;
	u8_headerBuffToSend[HEADER_FIELD_DEST] = u8_headerBuff[HEADER_FIELD_SOURCE];
	u8_headerBuffToSend[HEADER_FIELD_SIZE] = u8_size + 1; //Payload including message type
	u8_headerBuffToSend[HEADER_FIELD_CHECKSUM] = calcCheckSum(u8_headerBuffToSend, HEADER_BUFF_SIZE - 1);

	//RESET LINK
	bufferPutByte(&codecWriteBuffer, 0xAA);
	bufferPutByte(&codecWriteBuffer,0x55);

	//Header
	bufferPutN(&codecWriteBuffer, u8_headerBuffToSend, sizeof(u8_headerBuff));

	//Message Type
	bufferPutByte(&codecWriteBuffer,u8_function);

	//Data
	bufferPutN(&codecWriteBuffer,(uint8_t *)data, (int32_t)u8_size);

	//CRC
	bufferTemp[0] = u8_function;
	memcpy(&bufferTemp[1],data,u8_size);
	uint16_t u16_crc = crc16Calc(bufferTemp, (u8_size + 1));

	bufferPutByte(&codecWriteBuffer, (uint8_t)(u16_crc & 0xFF));
	bufferPutByte(&codecWriteBuffer, (uint8_t)(u16_crc >> 8));

	writeToStream(DATA_STREAM, &codecWriteBuffer);
}

static void SendResetLinkPackage(void)
{
	uint8_t u8_headerBuffToSend[HEADER_BUFF_SIZE];

	u8_headerBuffToSend[HEADER_FIELD_SOURCE] = GFD_ID;
	u8_headerBuffToSend[HEADER_FIELD_DEST] = u8_headerBuff[HEADER_FIELD_SOURCE];
	u8_headerBuffToSend[HEADER_FIELD_SIZE] = 1;
	u8_headerBuffToSend[HEADER_FIELD_CHECKSUM] = calcCheckSum(u8_headerBuffToSend, HEADER_BUFF_SIZE - 1);

	//synk
	bufferPutByte(&codecWriteBuffer, 0xAA);
	bufferPutByte(&codecWriteBuffer,0x55);

	//Header
	bufferPutN(&codecWriteBuffer, u8_headerBuffToSend, sizeof(u8_headerBuff));

	//payload
	bufferPutByte(&codecWriteBuffer, MESSAGE_RESET_LINK);

	//crc
	uint8_t resetlink = MESSAGE_RESET_LINK;
	uint16_t u16_crc = crc16Calc(&resetlink, sizeof(uint8_t));

	bufferPutByte(&codecWriteBuffer, (uint8_t)(u16_crc & 0xFF));
	bufferPutByte(&codecWriteBuffer, (uint8_t)(u16_crc >> 8));

	writeToStream(DATA_STREAM, &codecWriteBuffer);
}

static uint8_t calcCheckSum(void *data, uint8_t u8_size)
{
	uint8_t i = 0;
	uint8_t u8_checkSum = 0;

	while(u8_size--)
	{
		u8_checkSum += *(((uint8_t *)data) + i);
		i++;
	}

	return u8_checkSum;
}

void GFDProtocol_SendCommandExecutedPakage(uint8_t cmd)
{
	uint8_t u8_headerBuffToSend[HEADER_BUFF_SIZE];
	uint8_t buffTemp[2] = {0};

	u8_headerBuffToSend[HEADER_FIELD_SOURCE] = GFD_ID;
	u8_headerBuffToSend[HEADER_FIELD_DEST] = u8_headerBuff[HEADER_FIELD_SOURCE];
	u8_headerBuffToSend[HEADER_FIELD_SIZE] = 2;
	u8_headerBuffToSend[HEADER_FIELD_CHECKSUM] = calcCheckSum(u8_headerBuffToSend, HEADER_BUFF_SIZE - 1);

	//synk
	bufferPutByte(&codecWriteBuffer, 0xAA);
	bufferPutByte(&codecWriteBuffer,0x55);

	//Header
	bufferPutN(&codecWriteBuffer, u8_headerBuffToSend, sizeof(u8_headerBuff));

	//payload
	bufferPutByte(&codecWriteBuffer,cmd);
	bufferPutByte(&codecWriteBuffer,0x01);

	//CRC
	buffTemp[0] = cmd;
	buffTemp[1] = 0x01;

	uint16_t u16_crc = crc16Calc(buffTemp, sizeof(buffTemp));

	bufferPutByte(&codecWriteBuffer, (uint8_t)(u16_crc & 0xFF));
	bufferPutByte(&codecWriteBuffer, (uint8_t)(u16_crc >> 8));

	writeToStream(DATA_STREAM, &codecWriteBuffer);

	writeToStream(DATA_STREAM, &codecWriteBuffer);
}

static EN_GFDProtocolTaskState SynkState(void)
{
	EN_GFDProtocolTaskState en_nextState = SYNC;

	if(bufferGetLength(&codecReadBuffer) >= GFD_PREAMBULE_SIZE)
	{
		uint8_t temp_data;

		bufferGetByte(&codecReadBuffer, &temp_data);

		if (temp_data == GFD_PREAMBLE_HIGH) {

			bufferGetByte(&codecReadBuffer, &temp_data);

			//verifica byte 2 do preambulo
			if (temp_data == GFD_PREAMBLE_LOW) {

				en_nextState = GET_HEADER;

			} else {

				while(bufferGetByte(&codecReadBuffer,&temp_data) != -1);
			}
		}
	}

	return en_nextState;
}

static EN_GFDProtocolTaskState GetHeader(void)
{
	EN_GFDProtocolTaskState nextState = GET_HEADER;

	if(bufferGetLength(&codecReadBuffer) > HEADER_BUFF_SIZE)
	{
		bufferGetN(&codecReadBuffer, u8_headerBuff, (int32_t)HEADER_BUFF_SIZE);

		uint8_t u8_checkSum = calcCheckSum(u8_headerBuff, (uint8_t)(HEADER_BUFF_SIZE - 1));

		if(u8_checkSum == u8_headerBuff[HEADER_FIELD_CHECKSUM])
		{

		}

		nextState = GET_DATA;
	}

	return nextState;
}

static EN_GFDProtocolTaskState GetDataState(void)
{
	EN_GFDProtocolTaskState nextState = GET_DATA;

	if(bufferGetLength(&codecReadBuffer) > u8_headerBuff[HEADER_FIELD_SIZE])
	{
		bufferGetByte(&codecReadBuffer, &mu8_messageType);

		mu8_messageSize = u8_headerBuff[HEADER_FIELD_SIZE] - 1;

		if(mu8_messageSize > 0)
		{
			bufferGetN(&codecReadBuffer, mu8_messageBuff, (int32_t)mu8_messageSize);
		}

		if(mu8_messageType == 0x00)
		{
			uint8_t u8_temp = 0;
			SendResetLinkPackage();
			while(bufferGetByte(&codecReadBuffer, &u8_temp) != -1);

			nextState = SYNC;
		}

		nextState = CRC;
	}

	return nextState;
}

static EN_GFDProtocolTaskState CRCCheckState(void)
{
	EN_GFDProtocolTaskState nextState = SYNC;
	uint8_t temp;

	while(bufferGetByte(&codecReadBuffer, &temp) == 1);

	return nextState;
}

static void GFDProtocolTask(void)
{
	static EN_GFDProtocolTaskState en_taskState = SYNC;

	readStream(DATA_STREAM, &codecReadBuffer);

	switch(en_taskState)
	{
	case SYNC:

		en_taskState = SynkState();

		break;

	case GET_HEADER:

		en_taskState = GetHeader();

		break;

	case GET_DATA:

		en_taskState = GetDataState();

		break;
	case CRC:

		en_taskState = CRCCheckState();

		break;
	}
}

void GFDProtocol_Init(void)
{
	initBuffer(&codecReadBuffer, codecReadBufferArray, 64);
	initBuffer(&codecWriteBuffer, codecWriteBufferArray, 64);

	if(gfdProtocolTaskHandler != NULL)
		schedulerDelete(&gfdProtocolTaskHandler);

	schedulerSchedule(&gfdProtocolTaskHandler, 40, &GFDProtocolTask, true);
}

