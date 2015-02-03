/******************************************************************************
 *																			  *
 *                            CMSIS BASIC SCHEDULER							  *
 *																			  *
 ******************************************************************************
 * @file    scheduler.c														  *
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

/******************************************************************************
 *                                 I N C L U D E S                            *
 *****************************************************************************/

#include "stm32f4xx_conf.h"
#include "scheduler.h"
//#include "rtcbsp.h"
//#include "iwdg.h"

/*******************************************************************************
 *                              D E F I N I C O E S                             *
 *******************************************************************************/

#define SAMPLING_COUNTER 1

/*******************************************************************************
 *                                VARIAVEIS GLOBAIS                             *
 *******************************************************************************/

uint32_t currentTask; //tarefa sendo executada
uint32_t lastTaskRun;
uint32_t taskTimeCounter;

TTaskHandler tasks[MAX_TASKS]; //pool de tarefas

uint64_t base_time;				//relogio do sistema
uint32_t base_time_low;
uint32_t base_time_high;
uint32_t base_time_to_reset_wdt = 0;//40ms

int32_t sch_sampling_counter;

/*******************************************************************************
 *                                PROTOTIPOS FUNCOES                            *
 *******************************************************************************/

/*******************************************************************************
 *                                      FUNCOES                                *
 ******************************************************************************/

/**
 * void schedulerInit()
 *
 * Descricao - Prepara o scheduler para execucao
 *
 *
 * Parametros - taxa de incremento (tick)
 *
 *
 * Exemplo -
 *
 *
 */
void schedulerInit(bool b_wdtEnable) {

	if (b_wdtEnable == true) {
//		iwdg_Init();
//		iwdg_Start();

	}

	//inicializa RTC CLK 32.768KHz
	//rtcbsp_Init();

//	base_time = (uint64_t) (rtcbsp_GetUTCTime() * 1000LL);

	//Verificar se base_time > 09/2011
	if (base_time < 1314835200000LL) {
		base_time = 0;
	}

	int32_t i; //inicializa pool de tarefas
	for (i = 0; i < MAX_TASKS; i++) {
		tasks[i].state = FREE;
	}
	currentTask = 0;
	lastTaskRun = 99;

}

/**
 * @brief  This function schedule a task
 *
 * 		To use this function the Scheduler MUST be initialized first
 *
 * @param  uint32_t number_ticks: Interval in number of ticks that this
 * 		   		task will be executed.
 * 		    void(*task_func)(void): A pointer to function task
 *
 * @retval TTaskHandler*: A pointer to TTaskHandler stored in the pool of tasks
 *
 */
void schedulerSchedule(TTaskHandler **taskHandler, uint32_t number_ticks,
		void (*task_func)(void), bool run) {

	int32_t i;

	for (i = 0; i < MAX_TASKS; i++) {

		if (tasks[i].state == FREE) {

			tasks[i].identifier = i;
			tasks[i].tick_count = 0;
			tasks[i].number_ticks = number_ticks;
			tasks[i].function = task_func;

			if (run == true)
				tasks[i].state = SUSPENDED;
			else
				tasks[i].state = STOPPED;

			*taskHandler = (TTaskHandler*) &tasks[i];

			break;
		}
	}

}

/**
 * @brief
 *
 * @param
 *
 * @retval
 *
 */
void schedulerDelete(TTaskHandler **task) {

	TTaskHandler *handler = *task;

	if (handler != NULL ) {
		handler->state = FREE;
		*task = NULL;
	}

}

/**
 * @brief
 *
 * @param
 *
 * @retval
 *
 */
uint32_t schedulerScheduledTasks() {

	int32_t count = 0;
	int32_t i;

	for (i = 0; i < MAX_TASKS; i++) {
		if (tasks[i].state != FREE)
			count++;
		;
	}

	return count;

}

/**
 * void schedulerStart(void)
 * Descricao - Inicia o agendador de tarefas. Deve ser colocado do lugar
 * 			   do loop infinito
 * Parametros -
 * Exemplo -
 */
void schedulerRun(void) {

	while (1) {

		while (currentTask < MAX_TASKS) {

			//verifica se a proxima tarefa pode ser executada
			if (tasks[currentTask].state == READY) {

				tasks[currentTask].state = RUNNING;

				tasks[currentTask].function();

				if (tasks[currentTask].state == RUNNING)
					tasks[currentTask].state = SUSPENDED;

			}

			currentTask++;
		}

		if (currentTask >= MAX_TASKS) {
			currentTask = 0;

		}

	}
}

/**
 * @brief
 *
 * @param
 *
 * @retval None
 *
 */
void schedulerTick(void) {

	int32_t i;

	base_time++;


	if (currentTask != lastTaskRun) {
		lastTaskRun = currentTask;
		taskTimeCounter = 0;
	}

	//If the task delay more than 8ms
	if (taskTimeCounter++ > TIME_TO_RESET_DUE_BLOCKING_TASK_IN_MS) {

		reset();

	}

	//If the time elapsed is more than TIME_TO_REFRESH_WDT_RESET then refresh him
	base_time_to_reset_wdt++;
	if(base_time_to_reset_wdt == TIME_TO_REFRESH_WDT_RESET)
	{
		base_time_to_reset_wdt = 0;
//		iwdg_Reload(); // Reset iwdg
//		wdt_Refresh();
	}

	//incrementa tick de todas as tarefas agendadas
	for (i = 0; i < MAX_TASKS; i++) {
		if (tasks[i].state == SUSPENDED) {
			if (tasks[i].tick_count == tasks[i].number_ticks - 1) {
				tasks[i].tick_count = 0;
				tasks[i].state = READY;
			} else {
				tasks[i].tick_count++;
			}
		}
	}

}

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
void reset(void) {

	NVIC_SystemReset();

}

/**
 *
 */
void schedulerSetBaseTime(uint64_t time) {

	base_time = time;

//	rtcbsp_SetUTCTime((uint32_t) (time / 1000));

}

uint64_t schedulerGetBaseTime(void) {

	return base_time;

}

