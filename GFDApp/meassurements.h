/*
 * meassurements.h
 *
 *  Created on: 02/07/2013
 *      Author: Computador
 */

#ifndef MEASSUREMENTS_H_
#define MEASSUREMENTS_H_

void meassurements_Init(void);
void meassurements_Start();
float meassurements_GetCurrRMS(void);
float meassurements_GetVT(void);
float meassurements_GetVP(void);

#endif /* MEASSUREMENTS_H_ */
