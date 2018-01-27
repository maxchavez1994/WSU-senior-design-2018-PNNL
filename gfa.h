/*
 * GFA.h
 *
 *  Created on: March 2016
 *      Author: Aaron Miyasaki and Jacob Greig-Prine
 */
//***************************************************************************
//
//  Curent Version: $Revision$
//  Last Revision:  $Date$
//  Revised by:     $User$
//
//  Revision History:   See CVS Log
//
//***************************************************************************/
#include <msp430f2618.h>
#include "stdlib.h"
#include "stdio.h"
#include "math.h"
//#if !defined(HIPS_H__INCLUDED)  // Only include header file once
//#define HIPS_H__INCLUDED

#ifndef GFA_H_
#define GFA_H_

/* definitions */
typedef signed char int8;
typedef unsigned char uint8;
typedef signed short int16;
typedef unsigned short uint16;
typedef signed long int32;
typedef unsigned long uint32;

#define SER_BUFFER_SIZE  80

#define SYSCLK	 8000000		// SYSCLK / SPI_RATE should be evenly divisible

#define ISRFLG_TWOHZ    BIT2
#define ISRFLG_TIC_BIT 	BIT4
#define ISRFLG_USB_BIT  BIT6
#define ISRFLG_DMA_BIT  BIT5
#define ISRFLG_GPS_BIT  BIT7

#define ON		1
#define OFF 	0
#define TRUE	1
#define FALSE	0
#define ERR_OK 	0
#define CR		0x0d
#define BS		0x08

#define ERR_VALUE -1
#define DELAY_CONST 12
#define CODEC_RATE_ADDR	1
#define NUM_FREQ  15 // Number sample rate frequencies

#define Num_of_Results      2048

//void InitUart(void);
//void InitSystem(void);
//void InitTimers(void);
//void Set_DCO(void);

void freqCalc(volatile unsigned int voltage[Num_of_Results], int *offset, double frequency[Num_of_Results]);
double voltConv(volatile unsigned int voltage[Num_of_Results]);

#endif //  if !defined(GFA_H__INCLUDED)


// #endif /* GFA_H_ */
