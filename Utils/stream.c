/******************************************************************************
 *																			  *
 *                       Módulo de comunicação - ACCIO						  *
 *																			  *
 ******************************************************************************
 * FileName:        stream.c												  *
 * Dependencies:    Verificar INCLUDES										  *
 * Processor:       ARM7TDMI												  *
 * Compiler:        GCC         											  *
 * Company:         Altriz Ind. Com. Equip. Eletronicos Ltda				  *
 *																			  *
 * Description:		Neste arquivo estão as funções responsáveis pela controle *
 * 					das portas de comunicação do ACCIO. 					  *
 * 																			  *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * Author               Date        Comment									  *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * Daniel P Carvalho    10/06/09    Original.								  *
 *****************************************************************************/

/**
 * 	Changelog
 *
 * (06/10/2009)
 *
 *  * Acrescentados tarefa e buffer de escrita para o stream de dados.
 *
 * (07/10/2009)
 *
 *  * Modificada a função setPortStream() para permitir o ajuste da velocidade
 *    da serial.
 *  * Terminada a função que controla o sinal do PTT para comunicação via
 *    rádio
 *
 * 04/02/2011
 *
 * * Trocadas as definições de tipo de variáveis. Foi eliminado o uso do cabe-
 *   çalho "type.h", agora são utilizadas as definições padrão contidas em
 *   "stdint" e "stdbool"
 */

/******************************************************************************
 *                                 I N C L U D E S                            *
 *****************************************************************************/

#include "stream.h"
#include "debug.h"

//#include "LPC21XX_USART/src/uart.h"
#include "usart1bsp.h"
#include "usart3bsp.h"
#include "scheduler.h"
#include "usbd_cdc_core.h"
#include "usb_conf.h"

/******************************************************************************
 *                              D E F I N I C O E S                           *
 *****************************************************************************/

/******************************************************************************
 *                                VARIAVEIS GLOBAIS                           *
 *****************************************************************************/

int32_t uart1Stream = DEBUG_STREAM;
int32_t uart2Stream = DEBUG_STREAM;
int32_t usbStream = DEBUG_STREAM;

TTaskHandler *writeDataHandler;

TBuffer write_buffer; //Buffer usado para armazenar os dados
uint8_t write_buffer_array[512]; //a serem enviados

TBuffer write_buffer2; //Buffer usado para armazenar os dados
uint8_t write_buffer2_array[512]; //a serem enviados

TBuffer write_buffer3;     //buffer para armazenar os dados para DATA_STREAM_MON
uint8_t write_buffer3_array[512];

TBuffer write_buffer4; //buffer para armazenar os dados para DATA_STREAM_MON_AUX
uint8_t write_buffer4_array[512];

int32_t pre_tx_time; //tempo de pre tx
int32_t pre_tx_time_counter;

int32_t pre_tx_time2; //tempo de pre tx
int32_t pre_tx_time_counter2;

TBuffer write_usb_buffer; //Buffer usado para armazenar os dados
uint8_t write_usb_buffer_array[512]; //a serem enviados

TBuffer read_usb_buffer; //Buffer usado para armazenar os dados
uint8_t read_usb_buffer_array[512]; // recebidos



/******************************************************************************
 *                               PROTOTIPOS FUNCOES                           *
 *****************************************************************************/

/**
 * void prvWriteToDataStreamTask()
 *
 * Descricao - Tarefa encarregada de enviar os dados do
 * 			   buffer de saida de DATA STEAM
 *
 * Parametros -
 *
 * Retorno -
 *
 */
void prvWriteToDataStreamTask() {

	uint8_t data;

	if (bufferGetLength(&write_buffer) > 0) {

		//RTS1_PIN_LOW;

		if (pre_tx_time_counter-- == 0) {

			while (bufferGetByte(&write_buffer, &data) > 0) {
				//				u1Put(data);
				usart3bsp_SendNBytes(&data, 1);
			}

		}

	} else {

		pre_tx_time_counter = pre_tx_time;
		//RTS1_PIN_HIGH;

	}

}

/******************************************************************************
 *                                   FUNCOES  	                              *
 *****************************************************************************/

/**
 * void initCommunicationModule()
 *
 * Descricao - Inicializa o módulo de comunicação. Coloca o estado das portas
 * 			   de comunicação para seus valores padrão.
 *
 * Parametros -
 *
 * Retorno -
 *
 */
void initCommunicationModule() {

	//inicializa serial 0
	usart1bsp_Init(9600);
	uart1Stream = DEBUG_STREAM;

	usart3bsp_Init(9600);
	uart2Stream = DEBUG_STREAM;

	initBuffer(&write_buffer, (uint8_t*) &write_buffer_array, 512);
	initBuffer(&write_buffer2, (uint8_t*) &write_buffer2_array, 512);
	initBuffer(&write_buffer3, (uint8_t*) &write_buffer3_array, 512);
	initBuffer(&write_buffer4, (uint8_t*) &write_buffer4_array, 512);

	initBuffer(&write_usb_buffer, (uint8_t*) &write_usb_buffer_array, 512);
	initBuffer(&read_usb_buffer, (uint8_t*) &read_usb_buffer_array, 512);

	pre_tx_time = 0;
	pre_tx_time2 = 0;

	setDebugLevel(3);

}

/**
 * void setPortStream(int32_t port, int32_t stream)
 *
 * Descricao - Ajusta o stream associado às portas de comunicação
 *
 * Parametros - int32_t: indicando qual porta se deseja ajustar
 *				UINT: Stream que se deseja associar a porta
 *
 * Retorno -
 *
 */
void setPortStream(int32_t port, int32_t stream, int32_t speed) {

	if (port == COM1) {

		usart1bsp_ResetRxFIFO();
		uart1Stream = stream;

		usart1bsp_Init(speed);
	}

	if (port == COM2) {

		usart3bsp_ResetRxFIFO();

		uart2Stream = stream;

		usart3bsp_Init(speed);

	}
	if (port == USB) {

		usbStream = stream;
	}

}

/**
 * void setPreTxTime(int32_t time)
 *
 * Descricao - Ajusta o tempo de pre TX. O tempo é um múltiplo de
 * 				100 ms
 *
 * Parametros - int32_t: tempo
 *
 * Retorno -
 *
 */
void setPreTxTime(int32_t time) {

	if (time > 0) {

		pre_tx_time = time;
		//RTS1_PIN_HIGH;

		if (writeDataHandler != NULL )
			schedulerDelete(&writeDataHandler);
		schedulerSchedule(&writeDataHandler, 100, &prvWriteToDataStreamTask,
				true);

	} else {
		if (writeDataHandler != NULL )
			schedulerDelete(&writeDataHandler);
	}

}

void readStream(int32_t stream, TBuffer *buffer) {

	uint8_t data;

	if (uart1Stream == stream) {

		while (usart1bsp_GetNBytes(&data, 1) > 0) {
			bufferPutByte(buffer, data);

			if (uart2Stream == DATA_STREAM_MON) {
				usart3bsp_SendNBytes(&data, 1);
			}
		}
	}

	if (uart2Stream == stream) {

		while (usart3bsp_GetNBytes(&data, 1) > 0) {
			bufferPutByte(buffer, data);

			if (uart1Stream == DATA_STREAM_MON) {
				usart1bsp_SendNBytes(&data, 1);
			}
		}
	}
	if (usbStream == stream) {

		while (bufferGetByte(&read_usb_buffer, &data) > 0) {
					bufferPutByte(buffer, data);
				}
	}
}

/**
 * void writeToStream(int32_t stream, uint8_t *data, int32_t data_size)
 *
 * Descricao - Envia dados para as portas associadas ao fluxo desejado
 *
 * Parametros - int32_t: Fluxo que se deseja enviar os dados
 *				uint8_t*: ponteiro para os dados a serem escritos
 *				int32_t: quantidade de dados que seram escritos
 * Retorno -
 *
 */
void writeToStream(int32_t stream, TBuffer *buffer) {

	//FIXME: if the usart is busy, the data could not be transmitted.The data in circular
	//		 buffer will not be transmitted and will be lost.

	uint32_t nbr_of_bytes = bufferGetLength(buffer);

	if (nbr_of_bytes == 0) {
		return;
	}

	uint8_t data[BUFFERU1_TX_FIFO_SIZE];

	if (stream == DATA_STREAM) {

		if (uart1Stream == DATA_STREAM) {

			if (pre_tx_time > 0) {

				bufferGetN(buffer, data, nbr_of_bytes);
				bufferPutN(&write_buffer, data, nbr_of_bytes);

				if (uart2Stream == DATA_STREAM_MON) {
					bufferPutN(&write_buffer3, data, nbr_of_bytes);
				}

			} else {

				if (nbr_of_bytes > BUFFERU1_TX_FIFO_SIZE) {
					nbr_of_bytes = BUFFERU1_TX_FIFO_SIZE;
				}

				bufferGetN(buffer, data, nbr_of_bytes);

				usart1bsp_SendNBytes(data, nbr_of_bytes);

				if (uart2Stream == DATA_STREAM_MON) {
					if (nbr_of_bytes > BUFFERU3_TX_FIFO_SIZE) {
						nbr_of_bytes = BUFFERU3_TX_FIFO_SIZE;
					}

					usart3bsp_SendNBytes(data, nbr_of_bytes);
				}
			}
		}

		if (uart2Stream == DATA_STREAM) {
			if (pre_tx_time > 0) {
				bufferGetN(buffer, data, nbr_of_bytes);
				bufferPutN(&write_buffer, data, nbr_of_bytes);

				if (uart1Stream == DATA_STREAM_MON) {
					bufferPutN(&write_buffer3, data, nbr_of_bytes);
				}
			} else {
				if (nbr_of_bytes > BUFFERU3_TX_FIFO_SIZE) {
					nbr_of_bytes = BUFFERU3_TX_FIFO_SIZE;
				}

				bufferGetN(buffer, data, nbr_of_bytes);
				usart3bsp_SendNBytes(data, nbr_of_bytes);

				if (uart1Stream == DATA_STREAM_MON) {
					if (nbr_of_bytes > BUFFERU1_TX_FIFO_SIZE) {
						nbr_of_bytes = BUFFERU1_TX_FIFO_SIZE;
					}

					usart1bsp_SendNBytes(data, nbr_of_bytes);
				}
			}
		}

		if (usbStream == DATA_STREAM) {

			uint8_t u8_data;

			while (bufferGetByte(buffer, &u8_data) != -1) {

						VCP_TX(&u8_data, 1);

				}

		}

	} else if (stream == DATA_STREAM_AUX) {

		if (uart1Stream == DATA_STREAM_AUX) {
			if (pre_tx_time2) {
				bufferGetN(buffer, data, nbr_of_bytes);
				bufferPutN(&write_buffer2, data, nbr_of_bytes);

				if (uart2Stream == DATA_STREAM_AUX_MON) {
					bufferPutN(&write_buffer3, data, nbr_of_bytes);
				}

			} else {

				if (nbr_of_bytes > BUFFERU1_TX_FIFO_SIZE) {
					nbr_of_bytes = BUFFERU1_TX_FIFO_SIZE;
				}

				bufferGetN(buffer, data, nbr_of_bytes);

				usart1bsp_SendNBytes(data, nbr_of_bytes);

				if (uart2Stream == DATA_STREAM_AUX_MON) {
					if (nbr_of_bytes > BUFFERU3_TX_FIFO_SIZE) {
						nbr_of_bytes = BUFFERU3_TX_FIFO_SIZE;
					}

					usart3bsp_SendNBytes(data, nbr_of_bytes);
				}
			}

		}
		if (uart2Stream == DATA_STREAM_AUX) {

			if (pre_tx_time2 > 0) {

				bufferGetN(buffer, data, nbr_of_bytes);
				bufferPutN(&write_buffer2, data, nbr_of_bytes);

				if (uart1Stream == DATA_STREAM_AUX_MON) {
					bufferPutN(&write_buffer3, data, nbr_of_bytes);
				}

			} else {

				if (nbr_of_bytes > BUFFERU3_TX_FIFO_SIZE) {
					nbr_of_bytes = BUFFERU3_TX_FIFO_SIZE;
				}

				bufferGetN(buffer, data, nbr_of_bytes);

				usart3bsp_SendNBytes(data, nbr_of_bytes);

				if (uart1Stream == DATA_STREAM_AUX_MON) {
					if (nbr_of_bytes > BUFFERU1_TX_FIFO_SIZE) {
						nbr_of_bytes = BUFFERU1_TX_FIFO_SIZE;
					}

					usart1bsp_SendNBytes(data, nbr_of_bytes);
				}
			}

		}

	} else if (stream == DEBUG_STREAM) {

		uint32_t nbr_of_bytes = bufferGetLength(buffer);

		if (uart1Stream == DEBUG_STREAM) {

			if (nbr_of_bytes > BUFFERU1_TX_FIFO_SIZE) {
				nbr_of_bytes = BUFFERU1_TX_FIFO_SIZE;
			}

			bufferGetN(buffer, data, nbr_of_bytes);

			if (usart1bsp_SendNBytes(data, nbr_of_bytes) == 0) {

			}

		}

		if (uart2Stream == DEBUG_STREAM) {
			if (nbr_of_bytes > BUFFERU3_TX_FIFO_SIZE) {
				nbr_of_bytes = BUFFERU3_TX_FIFO_SIZE;
			}

			bufferGetN(buffer, data, nbr_of_bytes);

			if (usart3bsp_SendNBytes(data, nbr_of_bytes) == 0) {

			}
		}

		if(usbStream == DEBUG_STREAM)
		{
			uint8_t u8_data;

			while (bufferGetByte(buffer, &u8_data) != -1) {

				VCP_TX(&u8_data, 1);

			}
		}

	} else {

		//TODO: Erro!!!

	}

}

void streamDataReceived(uint8_t* buf, uint32_t len) {

	int32_t i = 0;

	for (i = 0; i < len; i++) {

		bufferPutByte(&read_usb_buffer, *buf++);
	}

}

/**
 * void flushStream(int32_t stream)
 *
 * Descricao - Esvazia o buffer de saída de um stream
 *
 * Parametros - int32_t: indicando qual stream deseja esvaziar
 *
 * Retorno -
 *
 */
void flushStream(int32_t stream) {

	if (uart1Stream == stream) {
		usart1bsp_ResetTxFIFO();
//	while (u0TxBufferIsEmpty() < 0);
	}

	if (uart2Stream == stream) {
		usart3bsp_ResetTxFIFO();
//	while (u1TxBufferIsEmpty() < 0);
	}

}

