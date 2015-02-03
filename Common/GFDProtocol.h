/*
 * GFDProtocol.h
 *
 *  Created on: 03/07/2013
 *      Author: Computador
 */

#ifndef GFDPROTOCOL_H_
#define GFDPROTOCOL_H_

#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#define MESSAGE_RESET_LINK				0x00
#define MESSAGE_ENABLE_FAULT_DETECT		0x01
#define MESSAGE_RELAY_STATE				0x02
#define MESSAGE_ENABLE_GENERATE_SIGNAL	0x03
#define MESSAGE_SET_GAIN				0x04
#define MESSAGE_SET_FREQ				0x05
#define MESSAGE_GET_CURRENT				0x06
#define MESSAGE_GET_FAULT_MASK			0x07
#define MESSAGE_COMMAND_EXECUTED		0xFE


void GFDProtocol_Init(void);
bool GFDProtocol_GetData(uint8_t messageType, void *data, uint8_t u8_size);
void GFDProtocol_SendData(uint8_t u8_function, void *data, uint8_t u8_size);
void GFDProtocol_SendCommandExecutedPakage(uint8_t cmd);

#endif /* GFDPROTOCOL_H_ */
