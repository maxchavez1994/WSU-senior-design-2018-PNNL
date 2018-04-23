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
 */

#include  <msp430x26x.h>
#include <stdlib.h>   // atoi()
#include <stdio.h>    // sprintf()
#include <string.h>
#include "Gfa.h"
#include "time.h"

#define UART_PRINTF

long averagePeriod(long AvgPeriod, long iPeriod);
int  Assign(char option, long value, int Count);
int fputc(int _c, register FILE * _fp);
int fputs(const char *_ptr, register FILE *_fp);
void InitSystem(void);
void InitTimers(void);
void InitUart(void);
int  ParseString(char * theString, int index);
void PrintDate(int month, int day, int year);
void PrintDec(long value);
void PrintTime(int hour, int min, int sec);
void PrintString(char * theString);
long RefineCrossing(volatile unsigned int voltage[Num_of_Results]);
void SerialReply(char option);
int  ServiceSerial(void);
int  Uart1TxChar(uint8 c);
void sample_ADC(struct Voltage *d);
void volt_zeroDetector(struct Voltage *d);
long period(struct Voltage d);

#define AVG_INTERVAL   4    // 2^4

// Global Variables
int TxCharCount = 0, TxReadIndex = 0, TxWriteIndex = 0;
int RxWriteIndex = 0, RxReadIndex = 0;
char TxBuffer[SER_BUFFER_SIZE], RxBuffer[SER_BUFFER_SIZE];
int ISR_Flag = 0;

tm_t clock_Time = {0, 0, 12, 22, 4, 2018, 1};   //(sec, min, hr, day, mon, year,0);

volatile unsigned int buffer0[Num_of_Results];
volatile unsigned int buffer1[Num_of_Results];

char message[SER_BUFFER_SIZE];



#define DEBUG 0

/****************************************************************************/
/*  Main function                                                           */
/*  Function : main                                                         */
/*      Parameters                                                          */
/*          Input   :  Nothing                                              */
/*          Output  :  Nothing                                              */
/****************************************************************************/
void main(void)
{
  struct Voltage Volts = {0,0,0,0,0};
  int LoopCount = 0;
  long Period = 910, tmpPeriod = 0, Period_ = 14560, PeriodAvg = (14560 << AVG_INTERVAL), ChargeCost = 0;
  int battCurrent = 0, battSOC = 20, TotWsec = 0;
  int iVolts = 115, iVolts_ = 115, aVolts = (115 << AVG_INTERVAL), Watts = 3000, Phase  = 0;
  int avgControl = 0;

  int i = 0;

  InitSystem();
  InitUart();
  InitTimers();                 // Enable tic timer

  while (1)
  {
    __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupts

    if (ISR_Flag & ISRFLG_USB_BIT)
    {		// Process UART interrupt
      ISR_Flag &= ~ISRFLG_USB_BIT;
      ServiceSerial();
    }
    if (ISR_Flag & ISRFLG_TWOHZ)
    {			// Increment LoopCount
      LoopCount++;
      ISR_Flag &= ~ISRFLG_TWOHZ;
    }
    if (ISR_Flag & ISRFLG_DMA_BIT)
    {
        TACCTL0 &= ~CCIE;
        ADC12CTL0 &= ~ENC;
        sample_ADC(&Volts);
        volt_zeroDetector(&Volts);
        tmpPeriod = period(Volts); //this function also has a check for good period values - resets period to 0 if not
        if (tmpPeriod >= 900) //checking for bad results again
        {
            avgControl++; //increment avgControl for finding the average next
            Period = tmpPeriod;
            Period_ += Period; //adding all known good periods together
            iVolts = Volts.delta; //setting iVolts to a known good delta
            iVolts_ += Volts.delta; //Add up delta values for averaging
        }
        if (avgControl == 16)
        {
            PeriodAvg = Period_ >> AVG_INTERVAL; //Divide Period_ by 16, to find the average period from the last 16 samples
            aVolts = iVolts_ >> AVG_INTERVAL; //same as PeriodAvg
            iVolts_ = 0;
            Period_ = 0;
        }



#if DEBUG

        /*printf("min: %d\r\n", Volts.min);
        printf("max: %d\r\n", Volts.max);
        printf("mid: %d\r\n", Volts.mid);
        printf("delta: %d\r\n", Volts.delta);
        /*for (i = 0; i < 4; i++)
        {
            printf("ZeroTM[%d]: %ld\r\n", i, Volts.zero_tm[i]);
        }*/
        /*printf("period: %ld\r\n", Period);
        printf("periodavg: %ld\r\n", PeriodAvg); */



#else
    	if (LoopCount++ > 20 && avgControl == 16)
    	{
    	    tm2time(&clock_Time);
		
    	    //printf("X= %ld, %ld, %d, %d, %d, %d.%d, %d.%d, %d, %d, %ld, ", Period, (PeriodAvg >> AVG_INTERVAL), battCurrent, battSOC, TotWsec, iVolts, 0, (aVolts >> AVG_INTERVAL), 0, Watts, Phase, ChargeCost);
            printf("X= %ld, %ld, %d, %d, %d, %d.%d, %d.%d, %d, %d, %ld, ", Period, PeriodAvg, battCurrent, battSOC, TotWsec, iVolts, 0, aVolts, 0, Watts, Phase, ChargeCost);
      	    printf("%02d:%02d:%02d %02d/%02d/%02d\r\n", (int)clock_Time.tm_hour, (int)clock_Time.tm_min, (int)clock_Time.tm_sec, (int)clock_Time.tm_mon, (int)clock_Time.tm_mday, (int)clock_Time.tm_year);
      	    LoopCount = 0;
      	    avgControl = 0;
    	}
#endif
        ISR_Flag &= ~ISRFLG_DMA_BIT;
        ADC12CTL0 |= ENC;
        TACCTL0 |= CCIE;
		clock_Time.tm_sec += 1;
        //    }

    }
  }  // while(1)
}  // main()

/**********************Sample_ADC********************************
 Description: Obtains min, max, delta, # of max and min data to populate the Voltage struct,
 which will then be used in the zero cross detector function.
 */


void sample_ADC(struct Voltage *d)
{
    int i = 0;


    d->min = 32767;
    d->max = 0;

    for (i = 0; i < Num_of_Results; i++)
    {
        if (buffer0[i] > d->max)
        {
            d->max = buffer0[i];

        }
        if (buffer0[i] < d->min)
        {
            d->min = buffer0[i];

        }
    }

    d->delta = d->max - d->min;

    d->mid = (d->delta >> 1) + d->min;

}

/*********************volt_zeroDetector****************************
Description: Detects changes between being above the mid point and below to then assign zero crossing time intervals.
 */

void volt_zeroDetector(struct Voltage *d)
{
    int i, j = 0, state = 0, state_last = 0, diff1, diff2;


        for (i = 1; i < Num_of_Results; i++)
        {                                                   // the first point always seems bad
            if (buffer0[i] > d->mid)       state = 1;    // set state = 1, positive - above mid point
            else if (buffer0[i] < d->mid)  state = -1;   // set state = -1, negative - below mid point
            else if (buffer0[i] == d->mid)  state = 0;   // set state = 0, when equal - at mid point
            if (i > 1)
            {
                if (state == 0)                 d->zero_tm[j++] = i;
                else if (state_last != state)   d->zero_tm[j++] = i;    // look for state transitions, but don't know if prior point was closer
            }
            state_last = state;
            if (j == 3) break;
        }
        for (j = 0; j < 3; j++)
        {
            if (buffer0[d->zero_tm[j]-1] > d->mid)   diff1 = buffer0[d->zero_tm[j]-1] - d->mid;
            else                                        diff1 = d->mid - buffer0[d->zero_tm[j-1]];
            if (buffer0[d->zero_tm[j]] > d->mid)     diff2 = buffer0[d->zero_tm[j]] - d->mid;
            else                                        diff2 = d->mid - buffer0[d->zero_tm[j]];
            if (diff1 < diff2)  d->zero_tm[j] -= 1;
        }
}

long period(struct Voltage d)
{
    long period = 0;

    period = d.zero_tm[2] - d.zero_tm[0];

    //detecting bad results
    if (period >= 900)
    {
        return period;
    }
    else
    {
        period = 0;
        return period;
    }
}





/************************************   Assign ******************************
Description: Assign takes the parsed commands and changes the operation of
             the instrument.
****************************************************************************/
int Assign(char option, long value, int Count)
{
  int status = ERR_OK;
  char message[SER_BUFFER_SIZE];

  switch (option)       {
    case 'd':  // Update Clock
    case 'D':
        if      (Count == 1)    clock_Time.tm_hour = (uint8)(value);
        else if (Count == 2)    clock_Time.tm_min  = (uint8)(value);
        else if (Count == 3)    clock_Time.tm_sec  = (uint8)(value);
        else if (Count == 4)    clock_Time.tm_mday = (uint8)(value);
        else if (Count == 5)    clock_Time.tm_mon  = (uint8)(value);
        else if (Count == 6)    clock_Time.tm_year = value;
        else if (Count == 7)    clock_Time.tm_isdst= (uint8)value;
        break;
      case 'f':             //  SampleRate
      case 'F':
//        TBCCR2 = (int)value;
//        TBCCR0 = (int)value;
        break;
      case 'h':             //  HeadPhone ON/OFF control
      case 'H':
//          if      (value == 1) P2OUT &= ~BIT6;
//          else if (value == 0) P2OUT |= BIT6;
          break;
      default:
          sprintf (message, "%c:%d Invalid Assignment\r\n", (char)option, (int)value);
          PrintString(message);
          status = ERR_VALUE;
          break;
  }
  return (status);
}

#ifdef UART_PRINTF
// ***************************************************************************
int fputc(int _c, register FILE *_fp)
{
    while (!(UC1IFG & UCA1TXIFG));
    UCA1TXBUF = (unsigned char) _c;
    return (unsigned char) _c;
}

//***************************************************************************
int fputs(const char *_ptr, register FILE *_fp)
{
    unsigned int i, len;

    len = strlen(_ptr);
    for (i = 0; i < len; i++)  {
        while (!(UC1IFG & UCA1TXIFG));
        UCA1TXBUF = (unsigned char) _ptr[i];
    }
    return (len);
}
#endif

/*****************************************************************************
    InitUART function - set constants & I/O
   Description: Echo a received character, RX ISR used. Normal mode is LPM0.
   USCI_A1 RX interrupt triggers TX Echo.
   Baud rate divider with 1MHz = 1MHz/115200 = ~8.7 - MUST CHANGE if DCOCTL changes
*****************************************************************************/
void InitUart(void) {
  P3SEL = 0xC0;                             // P3.6,7 = USCI_A1 TXD/RXD
  UCA1CTL1 |= UCSSEL_2;                     // SMCLK
  UCA1BR0 = 69;                              // 8MHz 115200
  UCA1BR1 = 0;                              // 8MHz 115200
  UCA1MCTL = UCBRS2;                        // Modulation UCBRSx = 4 for 8MHz
  UCA1CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
  UC1IE |= UCA1RXIE;                          // Enable USCI_A0 RX interrupt
}

/*****************************************************************************
    InitSystem function - set constants & I/O
*****************************************************************************/
void InitSystem(void) {
  unsigned int i;

  WDTCTL = WDTPW + WDTHOLD;     // Stop watchdog timer
  // set first functionality to all ports
  BCSCTL1 = CALBC1_8MHZ;                    // Set DCO to 8MHz
  DCOCTL = CALDCO_8MHZ;
  BCSCTL2 = 0;                  // SMCLK = MCLK = DCOCLK / 1
  for (i = 0xfffe; i > 0; i--);             // Delay for XTAL stabilization
  P1SEL = 0x00;
  P2SEL = 0x00;     // P2SEL must have BIT6/7 HIGH to enable 32kHz crystal, ACLK Active, P2.3 - P2.4 TA
  P3SEL = 0x00;
  P4SEL = 0x00;
  P5SEL = 0x00;
  P6SEL = BIT1 + BIT2;                      // Enable A/D on P6.1 and P6.2
  P7SEL = 0x00;
  P8SEL = 0x00;

  //set pin direction and initialize outputs
  P1DIR |= BIT0;                            // P1.0 output
//  P6DIR &= ~(BIT1);              // Set P6.1 as input
  TACCTL0 = 0x00;                // TACCR1 toggle, interrupt enabled
  TACCTL1 = 0x00;
  TACCTL2 = 0x00;
  TBCCTL0 = 0x00;
  TBCCTL1 = 0x00;
  TBCCTL2 = 0x00;
  ADC12CTL0 = REF2_5V + REFON + ADC12ON + MSC + SHT0_3;         // VRef 2.5V, Turn on ADC12, Multi-channel, sample hold 32 cycles
  ADC12CTL1 = SHS_3 + SHP + CONSEQ_3 + ADC12SSEL_0 + ADC12DIV_0;      // S&H TB.OUT1, rep. single chan, SMCLK
  ADC12MCTL0 = INCH_1 + SREF_1;                                 // Use A1 for P6.1
  ADC12MCTL1 = INCH_2 + SREF_1 + EOS;                           // Use A2 for P6.2

  ADC12IFG = 0x00;                                              // clear adc12 ifg
  ADC12CTL0 |= ENC;                                             // Enable ADC12

//  ADC12IE = 0x01;
  DMACTL0 = DMA0TSEL_6 + DMA1TSEL_6;
  DMA0SA = (void (*)())&ADC12MEM0;          // Src address = ADC12 module
  DMA0DA = (void (*)())&buffer0[0];         // Dst address
  DMA0SZ = Num_of_Results;                               // Size in words set to 500 in gfa.h
  DMA0CTL &= ~DMAIFG;                           //
  DMA0CTL = DMADSTINCR_3 + DMADT_4 + DMAIE + DMAEN;                // Sng rpt, config
  DMA1SA = (void (*)())&ADC12MEM1;          // Src address = ADC12 module
  DMA1DA = (void (*)())&buffer1[0];         // Dst address
  DMA1SZ = Num_of_Results;                               // Size of transfer
  DMA1CTL &= ~DMAIFG;                           //
  DMA1CTL = DMADSTINCR_3 + DMADT_4 + DMAIE + DMAEN;                // Sng rpt, config
}

/***************************************************************************
				       InitTimers
	Description: This function initializes the tic timer
	             TimerA is set as a up count timer with ACLK (32KHz)
	               as source, count up to value in TACCR0,
	               interrupt on compare (2 Hz)

	This must be called after SetDCO
***************************************************************************/
void InitTimers(void)
{
#define AdcRate 100
  TACTL = 0;
  TACTL = TASSEL_2 + TACLR + ID_3 + MC_1;      // SMCLK/8, clear TAR
  TACCTL0 = CCIE;                       // CCR0 interrupt enabled
  TACCR0 = 50000;
  TBCCR0 = AdcRate;                             // Init TBCCR0 in up mode w/ sample period (what are we setting the period to?)
  TBCCR1 = (AdcRate >> 1);                        // Trigger for ADC12 SC
  TBCCTL1 = OUTMOD_7;                       // Reset OUT1 on EQU1, set on EQU0
  TBCTL = TBSSEL_2 + MC_1 + TBCLR;          // SMCLK, clear TBR, up mode
}

/****************************************************************************
                            ParseString
Description: Separate leading character (command) and terminator '=' from
             the data.  Data is comma separated.
****************************************************************************/
int ParseString(char * theString, int index)
{
    long value;
    int Count = 0;
    char action[SER_BUFFER_SIZE + 1];
    char *pTest;

    pTest = strtok(theString, ",");
    do  {
        strncpy(&action[0], pTest, (unsigned long)index + 1);
        if (Count == 0) value = atoi(&action[2]);
        else            value = atoi(&action[0]);
        Assign((char)theString[0], value, Count);
        Count++;
        pTest = strtok(NULL,",");
    }   while (pTest != NULL);
    return (Count);
}

/****************************************************************************
							PrintDate
Description: Prints a formatted date.
****************************************************************************/
void PrintDate(int month, int day, int year)
{
	int i, j;
	char digit[11];

	digit[0] = ' ';
	digit[1] = '0' + (unsigned char) (month / 10);
	digit[2] = '0' + (unsigned char) (month % 10);
    digit[3] = '/';
	digit[4] = '0' + (unsigned char) (day / 10);
	digit[5] = '0' + (unsigned char) (day % 10);
    digit[6] = '/';
	digit[7] = '0' + (unsigned char) (year / 1000);
	i = year / 1000;
	i *= 1000;
	j = year - i;
	digit[8] = '0' + (unsigned char) (j / 100);
	i = j / 100;
	i *= 100;
	j -= i;
	digit[9] = '0' + (unsigned char) (j / 10);
    digit[10] = '0' + (unsigned char) (j % 10);
    strcat(message, digit);			//    PrintString(digit);		//
}

/****************************************************************************
							PrintDec
Description: Prints a long value as a decimal.
****************************************************************************/
void PrintDec(long value)
{
	#define MAX_DECIMAL	12

	int i, NegFlag = 0;
	long iInt = value;
	char digit[MAX_DECIMAL + 1];

	iInt = value;
	if (iInt & 0x80000000)		{
			iInt ^= 0xFFFFFFFF;
			iInt += 1;
			NegFlag = 1;
	}
    digit[MAX_DECIMAL] = '\0';
	for (i = MAX_DECIMAL - 1; i >= 0; i--)	{
		digit[i] = '0' + (unsigned char) (iInt % 10);
        iInt = iInt / 10;
        if (iInt == 0) break;
	}
    if (NegFlag)   digit[--i] = '-';
    strcat(message, &digit[i]); // PrintString(&digit[i]);

}

/****************************************************************************
							PrintTime
Description: Prints a formatted time.
****************************************************************************/
void PrintTime(int hour, int min, int sec)
{
	char digit[9];

    digit[0] = ' ';
    digit[1] = '0' + (unsigned char) (hour / 10);
    digit[2] = '0' + (unsigned char) (hour % 10);
    digit[3] = ':';
    digit[4] = '0' + (unsigned char) (min / 10);
    digit[5] = '0' + (unsigned char) (min % 10);
    digit[6] = ':';
    digit[7] = '0' + (unsigned char) (sec / 10);
    digit[8] = '0' + (unsigned char) (sec % 10);
    strcat(message, digit);     // PrintString(digit);
}


/******************************* PrintString ********************************
							
Description: Print null terminated ascii string to USB port
****************************************************************************/
void PrintString(char * theString)
{
  unsigned int i;
  int stringLength = strlen(theString);

  for (i = 0; i < stringLength; i++)	{
    if (theString[i] == '\0') break;
    while (!(UC1IFG & UCA1TXIFG));
    UCA1TXBUF = theString[i];
    // Uart1TxChar(theString[i]);
  }
}

// *********************** SerialReply **************************************
//  Provide a mechanism to query the system
// **************************************************************************
void SerialReply(char option)
{
  switch (option)
  {
      case 'p': // Recording Period Dwell
      case 'P':
          PrintString("P=");
          PrintString("%\r\n");
          break;
      default:
       PrintString("---\r\n");
       break;
  }
}

/****************************************************************************
                      ServiceSerial
Description: Process received USB data
****************************************************************************/
int ServiceSerial(void)
{
  static int menuFlag = 1, CommaCnt = 0;
  static uint8 theChar, index = 0;
  static char rxBuff[SER_BUFFER_SIZE];
  int status = 0;

  while (RxWriteIndex != RxReadIndex)   {
    theChar = RxBuffer[RxReadIndex];
    RxReadIndex = ((RxReadIndex + 1) & (SER_BUFFER_SIZE - 1));
    switch(theChar)
    {
      case 0x00:
          break;
      case CR:    // carriage return
          menuFlag = 0;
          PrintString("\r\n");
          rxBuff[index] = (char)0x00;   // Null terminate the string
          if (rxBuff[1] == '=') {
             status = ParseString((char *) rxBuff, index);
          }
          else if (rxBuff[1] == '?')    {
             SerialReply((char)rxBuff[0]);
          }
          else  status = -1;
            for (index = 0; index < (SER_BUFFER_SIZE - 1); index++) rxBuff[index] = '\0';
            index = 0;
            break;
      case ',':
          rxBuff[index] = (char)theChar;
          index++;
          CommaCnt++;
          Uart1TxChar(theChar);
          break;
      case BS:    // backspace
          if (index > 0)        {
            index--;
            rxBuff[index] = (char) 0x00;
          }
          Uart1TxChar(theChar);
          break;
      default:
          rxBuff[index] = (char) theChar;
          index++;
          index &= (SER_BUFFER_SIZE - 1);
          Uart1TxChar(theChar);
          break;
    }
    if (0 == menuFlag) break;
  } // while (RxWriteIndex != RxReadIndex);
  return ( (int) status);
}

//***************************************************************************
// Load TxBuffer and enable TX interrupt to send character out the USB Serial Port
//***************************************************************************
int Uart1TxChar(uint8 c)
{
  UC1IE &= ~UCA1TXIE;
  TxBuffer[TxWriteIndex++] = c;
  TxWriteIndex &= (SER_BUFFER_SIZE - 1);
//  TxCharCount++;
  UC1IE |= UCA1TXIE;
  return (int)TxWriteIndex;
}

//***************************************************************************
// Timer A1 interrupt service routine
//***************************************************************************
#pragma vector=TIMERA0_VECTOR
__interrupt void Timer_A (void)

{
  ISR_Flag |= ISRFLG_TWOHZ;
  __bic_SR_register_on_exit(LPM4_bits + GIE);     // Exit LPMx mode
}

//***************************************************************************
#pragma vector=USCIAB1RX_VECTOR
__interrupt void USCI1RX_ISR(void)
{
  ISR_Flag |= ISRFLG_USB_BIT;
  while (!(UC1IFG & UCA1TXIFG));                // USCI_A0 TX buffer ready?

  RxBuffer[RxWriteIndex++] = UCA1RXBUF;
  RxWriteIndex &= (SER_BUFFER_SIZE - 1);

  __bic_SR_register_on_exit(LPM4_bits + GIE);     // Exit LPMx mode
}

//***************************************************************************
#pragma vector=USCIAB1TX_VECTOR
__interrupt void USCI1TX_ISR(void)

{
  if (TxReadIndex != TxWriteIndex)  {
    UCA1TXBUF = TxBuffer[TxReadIndex++];
    TxReadIndex &= (SER_BUFFER_SIZE -1);
//    TxCharCount--;
  }
  else UC1IE &= ~UCA1TXIE;
  __bic_SR_register_on_exit(LPM4_bits + GIE);     // Exit LPMx mode
}

// Port 1 interrupt service routine *******************************
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
  P1IFG &= ~0x01;                          // P1.0 IFG cleared
  __bic_SR_register_on_exit(LPM4_bits + GIE);     // Exit LPMx mode
}

//***************************************************************************
// DMA interrupt service routine
#pragma vector = DMA_VECTOR
__interrupt void DMA_ISR(void)
{
    P1OUT ^= BIT0;
    switch(__even_in_range(DMAIV,16))
    {
      case 0: break;
      case 2:                                 // DMA0IFG = DMA Channel 0
        ISR_Flag |= ISRFLG_DMA_BIT;
        break;
      case 4: break;                          // DMA1IFG = DMA Channel 1
      case 6: break;                          // DMA2IFG = DMA Channel 2
      case 8: break;                          // DMA3IFG = DMA Channel 3
      case 10: break;                         // DMA4IFG = DMA Channel 4
      case 12: break;                         // DMA5IFG = DMA Channel 5
      case 14: break;                         // DMA6IFG = DMA Channel 6
      case 16: break;                         // DMA7IFG = DMA Channel 7
      default: break;
    }
   DMA0CTL &= ~DMAIFG;         // Clear DMA IFG

  __bic_SR_register_on_exit(LPM4_bits + GIE);     // Exit LPMx mode
}
