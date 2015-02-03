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
 * Description:		Neste arquivo estão as funções responsáveis pelo controle *
 * 					das mensagens de DUBUG do ACCIO		 					  *
 * 																			  *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * Author               Date        Comment									  *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * Daniel P Carvalho    10/06/09    Original.								  *
 *****************************************************************************/

/**
 * 	Changelog
 *
 * 04/02/2011
 *
 * * Trocadas as definições de tipo de variáveis. Foi eliminado o uso do cabe-
 *   çalho "type.h", agora são utilizadas as definições padrão contidas em
 *   "stdint" e "stdbool"
 *
 *
 */

/******************************************************************************
 *                                 I N C L U D E S                            *
 *****************************************************************************/

#include "debug.h"
#include "stream.h"

#include "buffer.h"



/******************************************************************************
 *                              D E F I N I C O E S                           *
 *****************************************************************************/


/******************************************************************************
 *                                VARIAVEIS GLOBAIS                           *
 *****************************************************************************/
TBuffer debugBuffer; 		//buffer de saida
uint8_t debugBufferArray[512];	//

int32_t debug_level = 3;


/******************************************************************************
 *                               PROTOTIPOS FUNCOES                           *
 *****************************************************************************/


/******************************************************************************
 *                                   FUNCOES  	                              *
 *****************************************************************************/

/**
 * void setDebugLevel(int32_t level)
 *
 * Descricao - Ajusta o valor do nível de debug.
 *
 * Parametros - int32_t: nivel desejado do debug
 *
 * Retorno -
 *
 */
void setDebugLevel(int32_t level) {

	initBuffer(&debugBuffer, (uint8_t*)&debugBufferArray[0], 128);
	debug_level = level;

}


/**
 * void printDebug(int32_t level, uint8_t *msg)
 *
 * Descricao - Imprime uma mensagem de debug.
 *
 * Parametros - int32_t: nivel desejado do debug
 * 				uint8_t*: string terminada em \0 que se deseja imprimir
 *
 * Retorno -
 *
 */
void printDebug(int32_t level, char *msg) {

	if(level >= debug_level) {

		while(*msg != '\0')
			bufferPutByte(&debugBuffer, *msg++);

		writeToStream(DEBUG_STREAM, &debugBuffer);

	}

	flushStream(DEBUG_STREAM);

}
