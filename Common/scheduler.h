/******************************************************************************
 *																			  *
 *                            CMSIS BASIC SCHEDULER							  *
 *																			  *
 ******************************************************************************
 * @file    scheduler.h														  *
 * @author  Altriz Ind. Com. Equip. Eletrônicos								  *
 * @version V1.0.0															  *
 * @date    24-July-2012													  *
 * @brief   																  *
 ******************************************************************************
 * @attention																  *
 *																			  *
 *																			  *
 *																			  *
 * <h2><center>&copy; COPYRIGHT 2012 Altriz</center></h2>					  *
 *****************************************************************************/

/**
 * 	Changelog
 *
 * v0.1
 *
 */

#ifndef SCHEDULER_H_
#define SCHEDULER_H_

/*******************************************************************************
*                                 I N C L U D E S                              *
*******************************************************************************/

/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/*******************************************************************************
*                              D E F I N I C O E S                             *
*******************************************************************************/

#define MAX_TASKS   18				//numero maximo de tarefas

#define _10US_60MHZ 	600				//frequencia que o agendador e chamado 10US
#define _1MS_60MHZ 		60000			//frequencia que o agendador e chamado 1MS
#define _10MS_60MHZ		600000			//frequencia que o agendador e chamado 10MS
#define _100MS_60MHZ	6000000			//frequencia que o agendador e chamado 100MS
#define _1000MS_60MHZ	60000000		//frequencia que o agendador e chamado 1000MS

#define RUNNING 	0
#define READY 		1
#define SUSPENDED	2
#define BLOCKED 	3
#define STOPPED 	4
#define FREE 	99

#define TIME_TO_RESET_DUE_BLOCKING_TASK_IN_MS	8000
#define TIME_TO_REFRESH_WDT_RESET	1000

typedef struct {
	int32_t identifier;
	int32_t state;
	int32_t tick_count;
	int32_t number_ticks;
	void (*function)(void);
}TTaskHandler;


/*******************************************************************************
*                              PROTOTIPO DE FUNCOES                            *
*******************************************************************************/

/**
 * void schedulerInit()
 *
 * Descricao - Inicializa o agendador de tarefas
 *
 * Parametros - int32_t tick_rate: frequencia com que os contadores das tarefas
 * 							   sao incrementados
 * 				bool wdt_enable: flag que habilita o watchdog timer
 *
 * Retorno -
 *
 */
void schedulerInit(bool b_wdtEnable);

/**
 * @brief  Adiciona uma tarefa à fila de execução. Antes de utilizar esta
 *  função é necessário inicializar o Scheduler através da função schedulerInit
 *
 * @param  uint32_t number_ticks: Interval in number of ticks that this
 * 		   		task will be executed.
 * 		    void(*task_func)(void): A pointer to function task
 *
 * @retval
 *
 */
void schedulerSchedule(TTaskHandler **taskHandler, uint32_t number_ticks,
		void (*task_func)(void), bool run);

/**
 * @brief
 *
 * @param
 *
 * @retval
 *
 */
void schedulerDelete(TTaskHandler **task);

/**
 * @brief
 *
 * @param
 *
 * @retval
 *
 */
uint32_t schedulerScheduledTasks();

/**
 * void schedulerRun(void)
 *
 * Descricao - Inicia o agendador de tarefas
 *
 * Parametros -
 *
 * Retorno -
 *
 */
void schedulerRun(void);

/**
 * @brief
 *
 * @param
 *
 * @retval None
 *
 */
void schedulerTick(void);




/**
 * void reset()
 *
 * Descricao - Interrompe a atualização do WDT forçando o reset
 *             do sistema.
 *
 * Parametros -
 *
 * Retorno -
 *
 */
void reset();

void schedulerSetBaseTime(uint64_t time);
//void sch_setBaseTime(uint32_t timer_low, uint32_t timer_high);
uint64_t schedulerGetBaseTime();
//void sch_getBaseTime(uint32_t *timer_low, uint32_t *timer_high);

/**
 * @brief
 *
 * @param
 *
 * @retval None
 *
 */
void schedulerTick(void);

#endif /*SCHEDULER_H_*/
