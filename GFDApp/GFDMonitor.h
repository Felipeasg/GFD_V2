/*
 * GFDMonitor.h
 *
 *  Created on: 03/07/2013
 *      Author: Computador
 */

#ifndef GFDMONITOR_H_
#define GFDMONITOR_H_

//valor entre 0 e 1 onde a hysterese para deteccao de falta em porcentagem
/*
 *         Hyseresi para diser se  falta ou nao
 *       |<----------->|
 *  |----*------|------*-----|
 *  0          VT/2			VT
 *
 *  esta hysterese afeta a sensibilidade do detector se ela for 0.25 quer dizer ela vai usar uma margem
 *  de 25% antes e depois de VT para falar se tem falta ou nao. Se a tensao for maior que 25% de VT ou menor
 *  que 75% de VT uma falta sera acusada sendo que de 0% ate 25% significa falta no barramento positivo
 *  e de 75% ate VT indica falta no barramento negativo.
 * */
#define HYSTERESIS_TO_DETECT_FAULT	0.3F
#define MIN_TO_CONSIDERERE_BUS_ACTIVE 0.1	//se estiver muito baixo ou o barramento esta em curto ou
//ainda nao foi conectado o barramento na placa

void GFDMonitor_Init(void);


#endif /* GFDMONITOR_H_ */
