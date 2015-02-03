/******************************************************************************
 *																			  *
 *                       Módulo de comunicação - ACCIO						  *
 *																			  *
 ******************************************************************************
 * FileName:        communications.h										  *
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
 * (07/10/2009)
 *
 * Alterado o protótipo da função setPortStream() para permitir o ajuste de
 * velocidade de comunicação.
 *
 * 04/02/2011
 *
 * * Trocadas as definições de tipo de variáveis. Foi eliminado o uso do cabe-
 *   çalho "type.h", agora são utilizadas as definições padrão contidas em
 *   "stdint" e "stdbool"
 *
 *
 */

#ifndef STREAM_H_
#define STREAM_H_

/******************************************************************************
 *                                 I N C L U D E S                            *
 *****************************************************************************/

/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "buffer.h"

/******************************************************************************
 *                              D E F I N I C O E S                           *
 *****************************************************************************/

#define		COM1			0
#define		COM2			1
#define		USB			2

#define		DEBUG_STREAM		0
#define 	DATA_STREAM			1
#define 	DATA_STREAM_MON		2
#define 	DATA_STREAM_AUX		3
#define 	DATA_STREAM_AUX_MON	4

#define 	BUFFERU1_TX_FIFO_SIZE	T_BUFFER_TX_SIZE_U1
#define		BUFFERU3_TX_FIFO_SIZE	T_BUFFER_TX_SIZE_U3

/******************************************************************************
 *                               PROTOTIPOS FUNCOES                           *
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
void initCommunicationModule();

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
void setPortStream(int32_t port, int32_t stream, int32_t speed);

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
void setPreTxTime(int32_t time);

void readStream(int32_t stream, TBuffer *buffer);

/**
 * void writeToStream(int32_t stream, BYTE *data, int32_t data_size)
 *
 * Descricao - Envia dados para as portas associadas ao fluxo desejado
 *
 * Parametros - int32_t: Fluxo que se deseja enviar os dados
 *				BYTE*: ponteiro para os dados a serem escritos
 *				int32_t: quantidade de dados que seram escritos
 * Retorno -
 *
 */
void writeToStream(int32_t stream, TBuffer *buffer);

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

void streamDataReceived(uint8_t* buf, uint32_t len);

void flushStream(int32_t stream);


#endif /* STREAM_H_ */
