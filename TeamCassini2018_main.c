/*
 * main.c
 *
 *  Created on: March 2016
 *      Author: Aaron Miyasaki and Jacob Greig-Prine
 *  Last Edited: April 2017
 *      Author: James Bowen and Cameron Bailey
 *
 *  MSP430 USERS_MANUAL URL
 *    http://www.ti.com/lit/ug/slau049f/slau049f.pdf
 *
 *      Revised: R. Pratt - 10/24/2017
 *      Revised: D. Yruretagoyena - 4/22/2018
 *      Revised: D. Yruretagoyena - 8/9/2018
 */

/* Design Requirements Reminder:
   ± 0.001Hz frequency resolution over the range from 45 to 65 Hz
   100 – 270VAC with 0.05VAC resolution
   A.C. current from 500mA to 30A with 10mA resolution
   Measure phase ±10 Degrees
   Measurement update rate faster than 1Hz
 *
 *
 *
 * Remaining Code for Fall 2018 EE416
 *
 *Frequency using comparator and timers.
 */

#include <msp430x26x.h>
#include <stdlib.h>   // atoi()
#include <stdio.h>    // sprintf()
#include <string.h>
//#include "Gfa.h"
//#include "time.h"
#include "functions.h"

#define DEBUG 0


/*******************************************************************************************/
/*  Main function                                                                          */
/*  Function : main                                                                        */
/*      Parameters                                                                         */
/*          Input   :  60hz Sinusoidal Signal from a load or function generator(debug)     */
/*          Output  :  Transmits a string of signal data through the UART to the GUI       */
/*******************************************************************************************/
void main(void)
{
  struct Voltage Volts = {0,0,0,0,0,0};
  struct Current Amps = {0,0,0,0,0,0};
  tm_t clock_Time = {0, 0, 12, 26, 10, 2018, 1}; //(sec, min, hr, day, mon, year,0);
  int LoopCount = 0;
  long Period = 910, tmpPeriod = 0, tmp_CurrPeriod = 0, Period_ = 14560, PeriodAvg = (14560 << AVG_INTERVAL), ChargeCost = 0;
  int battCurrent = 0, battSOC = 20, TotWsec = 0;
  int iVolts = 115, Watts = 3000, Phase, AvgPhase  = 0;
  long iVolts_ = 115, aVolts = (115 << AVG_INTERVAL);
  long iAmps_ = 0;
  int avgControl = 0;

  InitSystem();
  InitUart();
  InitTimers(); //Enable tic timer
  InitDAC();    //Using DAC for op-amp(s) offset
  SetupCompA();

  while (1)
  {
    __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupts

    if (ISR_Flag & ISRFLG_USB_BIT)
    {		// Process UART interrupt
      ISR_Flag &= ~ISRFLG_USB_BIT;
      ServiceSerial();
    }
    if (ISR_Flag & ISRFLG_TWOHZ)
    {	    // Increment LoopCount
      LoopCount++;
      ISR_Flag &= ~ISRFLG_TWOHZ;
    }
    if (ISR_Flag & ISRFLG_DMA_BIT) // & CompBuffer_Flag??
    {
        //If ISR_Flag DMA bit flag and CompBuffer_Flag == 1 - Do the following?
        TACCTL0 &= ~CCIE; //disabled TimerA
        ADC12CTL0 &= ~ENC; //Disable ADC
        //CACTL1 &= ~CAON; //Disable comparator
        avgControl++;
        //FREQUENCY
        //Calculate "Period" from CompVolts buffer values
        //PERIOD HERE
        //PeriodAvg += (556350 + CompBuffer[32]) - CompBuffer[0]; //55635 * 10(# of timer roll overs prior to index 32) + index 32 value (For total time to "zerocross" 32) subtracted by Buffer index 0 value for time between first and 32nd "zerocross"
        //Data collection from ADCs
        sample_signals(&Volts, &Amps);
        //Call Accuracy Improvement Functions
        ImproveVoltageAccuracy(&Volts);
        ImproveCurrentAccuracy(&Amps);
        //Obtain voltage and current
        iVolts = Volts.delta; //setting iVolts to a good delta
        iVolts_ += Volts.delta; //Add up delta values for averaging
        iAmps_ += Amps.delta; //Add up delta current values for averaging
        //Phase
        //AveragePeakIndices(&Volts, &Amps); //this is redundant
        AvgPhase += FindPhase(Volts, Amps);

        if (avgControl == 16)
        {
            //PeriodAvg = Period_ >> AVG_INTERVAL; //Divide Period_ by 16, to find the average period from the last 16 samples
            aVolts = iVolts_ >> AVG_INTERVAL; //same as PeriodAvg
            battCurrent = iAmps_ >> AVG_INTERVAL;
            Phase = AvgPhase >> AVG_INTERVAL;
            //Reset variables used for accumulating values for averaging
            iVolts_ = 0;
            iAmps_ = 0;
            //Period_ = 0;
            AvgPhase = 0;
        }



#if DEBUG

        /*printf("min: %d\r\n", Volts.min);
        printf("max: %d\r\n", Volts.max);
        printf("mid: %d\r\n", Volts.mid);
        printf("delta: %d\r\n", Volts.delta);*/
        /*for (i = 0; i < 4; i++)
        {
            printf("ZeroTM[%d]: %ld\r\n", i, Volts.zero_tm[i]);
        }*/
        /*printf("period: %ld\r\n", Period);
        printf("periodavg: %ld\r\n", PeriodAvg); */

        //printf("%d", numCrossings);



#else
    	if (LoopCount++ > 20 && avgControl == 16)
    	{
    	    tm2time(&clock_Time);

    	    //printf("X= %ld, %ld, %d, %d, %d, %d.%d, %d.%d, %d, %d, %ld, ", Period, (PeriodAvg >> AVG_INTERVAL), battCurrent, battSOC, TotWsec, iVolts, 0, (aVolts >> AVG_INTERVAL), 0, Watts, Phase, ChargeCost);
            printf("X= %ld, %ld, %d, %d, %d, %d.%d, %d.%d, %d, %d, %ld, ", Period_, PeriodAvg, battCurrent, battSOC, TotWsec, iVolts, 0, aVolts, 0, Watts, Phase, ChargeCost);
      	    printf("%02d:%02d:%02d %02d/%02d/%02d\r\n", (int)clock_Time.tm_hour, (int)clock_Time.tm_min, (int)clock_Time.tm_sec, (int)clock_Time.tm_mon, (int)clock_Time.tm_mday, (int)clock_Time.tm_year);
      	    LoopCount = 0;
      	    avgControl = 0;
    	}
#endif

        ISR_Flag &= ~ISRFLG_DMA_BIT;
        ADC12CTL0 |= ENC; //Re-enable ADC12
        TACCTL0 |= CCIE; //Re-enable TIMERA
        //CACTL1 |= CAON; //Re-enable Comparator
        CompVolts[33] = {0} //Re-initialize buffer to 0s
		clock_Time.tm_sec += 1; //increment time stamp*/

    }
  }  // while(1)
}  // main()

