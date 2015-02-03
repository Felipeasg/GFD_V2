/******************************************************************************
 *																			  *
 *                       Módulo de depuração - ACCIO						  *
 *																			  *
 ******************************************************************************
 * FileName:        debug.h													  *
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
 * 04/02/2011
 *
 * * Trocadas as definições de tipo de variáveis. Foi eliminado o uso do cabe-
 *   çalho "type.h", agora são utilizadas as definições padrão contidas em
 *   "stdint" e "stdbool"
 *
 *
 */

#ifndef DEBUG_H_
#define DEBUG_H_

/******************************************************************************
 *                                 I N C L U D E S                            *
 *****************************************************************************/

/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/******************************************************************************
 *                              D E F I N I C O E S                           *
 *****************************************************************************/


/******************************************************************************
 *                               PROTOTIPOS FUNCOES                           *
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
void setDebugLevel(int32_t level);

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
void printDebug(int32_t level, char *msg);

extern char __escape_assert_buffer[];

#define ESCAPE_ASSERT_WITH_ID(file,line) \
	sprintf(__escape_assert_buffer,"esc asrt: %s %d;\r\n",file,line); \
	printDebug(3,__escape_assert_buffer); \
	while(1);


#define ESCAPE_ASSERT 	ESCAPE_ASSERT_WITH_ID(__FILE__,__LINE__)


#endif /* DEBUG_H_ */
