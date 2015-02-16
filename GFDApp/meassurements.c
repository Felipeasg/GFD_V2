/*
 * meassurements.c
 *
 *  Created on: 02/07/2013
 *      Author: Computador
 */

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "adcbsp.h"

#define FS 10000.0
#define T  0.04
#define Fop 13

#define ACS_GAIN 17.837837837*17.837837837

#define AD_VT_GAIN	3.3/4095
#define AD_VP_GAIN	3.3/4095

static float a = 0;
float in, out[2];

float x[3];
float y[3];

uint32_t u32_adVp = 0;
uint32_t u32_adVt = 0;

static float k = 0;

//ACS712 -> 185mV - 1A
/*
 * Como temos um filtro passa alta na entrada e o sinal do ad � unit�rio pois � dividio por
 * 4095 logo na entrada do filtro temos que um sinal de 3.3 volts senoidal
 * variando de 0 - 3.3V na entrada do ad ser� um sinal variando senoidal variando de -0.5 - 0.5
 * na saida do filtro digital.
 * -> 0.5 equivale ent�o a 3.3V na entrada e -0.5 equivale a 0V
 * -> no ACS712 temos que para cada 185mV - 1A
 *
 * Como a referencia est� em 1.65V e valor m�ximo � 3.3V o valor m�ximo de corrente
 * medido ser� correspondente a 1.65V
 *
 * logo 3.3V  = 17.837837837 A
 *      1.65V = 8.9189489185 A
 *
 * ou seja       0.5      ---- 8.9189489185
 *         x[ADCfiltrado] ---- x[A]
 *
 *         x[A] =  x[ADCfiltrado]*8.91894891857
 *                --------------------
 *                        0.5
 *         x[A] = 17.837837837*x[ADCfiltrado]
 *
 *
 *         1 -
 */

//http://ieeexplore.ieee.org/xpl/login.jsp?tp=&arnumber=5941002&url=http%3A%2F%2Fieeexplore.ieee.org%2Fiel5%2F19%2F5978243%2F05941002.pdf%3Farnumber%3D5941002
//http://music.columbia.edu/pipermail/music-dsp/2007-June/066267.html
//http://www.analog.com/en/content/scientist_engineers_guide/fca.html //dsp guide

static void meassurent_handler(volatile uint32_t* smp);

float fastabs(float f)
{int i=((*(int*)&f)&0x7fffffff);return (*(float*)&i);}

void meassurements_Init(void)
{
	a = expf(-(1/(FS*T)));

	k = powf(2,-Fop);

	adcbsp_Init();
	adcbsp_RegisterADCHandler(meassurent_handler);
}

void meassurements_Start()
{
	adcbsp_StartSamples();
}

static void meassurent_handler(volatile uint32_t* smp)
{
	u32_adVp = *(smp + 0);
	u32_adVt = *(smp + 1);
	//in = fastabs(((float)(*smp))/4095);
//	(smp + 2) - IN3 ADC3 -> PIN PA3

	//Filtro para frequencia de amostragem de 10000 e frequencia de corte em 60HZ
	x[0] = x[1];
	x[1] = x[2];
	x[2] = ((*(smp+2)/4095.0));


	y[2] = x[0] - (2*x[1]) + x[2] - (0.9565436765 * y[0]) + (1.9555782403 * y[1]);

	y[0] = y[1];
	y[1] = y[2];


	in = y[2] * y[2] * ACS_GAIN;

//	out[1] = (1*in - out[0]);
	out[1] = ((1*in - out[0])*k) + out[0];

	out[0] = out[1];
	//Calculo do RMS utilizando um passa baixa no sinal de entrada ao quadrado
	/*in = y[2]*y[2]*ACS_GAIN;

	out[1] = sqrt(in + a*((out[0]*out[0]) - in));

	out[0] = out[1];*/

}

float meassurements_GetCurrRMS(void)
{
	return sqrtf(out[1]);
}

float meassurements_GetVT(void)
{
	return (float)(u32_adVt)*AD_VT_GAIN;
}

float meassurements_GetVP(void)
{
	return (float)(u32_adVp)*AD_VP_GAIN;
}


