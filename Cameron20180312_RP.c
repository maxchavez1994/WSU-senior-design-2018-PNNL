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
int  voltConv(volatile unsigned int voltage[Num_of_Results]);
long volt_zeroDetector(volatile unsigned int voltage[Num_of_Results]);

#define AVG_INTERVAL   4    // 2^4

// Global Variables
int TxCharCount = 0, TxReadIndex = 0, TxWriteIndex = 0;
int RxWriteIndex = 0, RxReadIndex = 0;
char TxBuffer[SER_BUFFER_SIZE], RxBuffer[SER_BUFFER_SIZE];
int ISR_Flag = 0;

tm_t clock_Time = {0, 0, 12, 7, 11, 2016, 1};   //(sec, min, hr, day, mon, year,0);

volatile unsigned int buffer0[Num_of_Results];
volatile unsigned int buffer1[Num_of_Results];

char message[SER_BUFFER_SIZE];

struct Voltage Volts = {0, 0, 0, 0, 0, 0, 0};

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
  int LoopCount = 0;
  long Period = 830, PeriodAvg = (13280 << AVG_INTERVAL), ChargeCost = 0;
  int battCurrent = 0, battSOC = 20, TotWsec = 0;
  int iVolts = 115, aVolts = (115 << AVG_INTERVAL), Watts = 3000, Phase  = 0;

  InitSystem();
  InitUart();
  InitTimers();                 // Enable tic timer

  while (1) {
    __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupts

    if (ISR_Flag & ISRFLG_USB_BIT)  {		// Process UART interrupt
      ISR_Flag &= ~ISRFLG_USB_BIT;
      ServiceSerial();
    }
    if (ISR_Flag & ISRFLG_TWOHZ) {			// Increment LoopCount
      LoopCount++;
      ISR_Flag &= ~ISRFLG_TWOHZ;
    }
    if (ISR_Flag & ISRFLG_DMA_BIT) {
        TACCTL0 &= ~CCIE;
        ADC12CTL0 &= ~ENC;
        iVolts = voltConv(buffer0);  // determines the max, min, delta, and mid voltage value
        Period = PeriodAvg >> AVG_INTERVAL;
        PeriodAvg -= Period;
    	volt_zeroDetector(buffer0);
//		Period += RefineCrossing(buffer0);
//		PeriodAvg += Period;
//		RefineCrossing(buffer0);
#if DEBUG
    	printf("mid: %d max: %d min: %d delta: %d Period: %ld AvgPeriod: %ld\r\n", Volts.mid, Volts.peak, Volts.min, Volts.delta, Period, PeriodAvg);
#else
    	if (LoopCount++ > 20)  {
    	    tm2time(&clock_Time);
		
//    	    printf("X= %ld, %ld, %d, %d, %d, %d.%d, %d.%d, %d, %d, %ld, ", Period, (PeriodAvg >> AVG_INTERVAL), battCurrent, battSOC, TotWsec, iVolts, 0, (aVolts >> AVG_INTERVAL), 0, Watts, Phase, ChargeCost);
            printf("X= %ld, %ld, %d, %d, %d, %d.%d, %d.%d, %d, %d, %ld, ", PeriodAvg, PeriodAvg, battCurrent, battSOC, TotWsec, iVolts, 0, (aVolts >> AVG_INTERVAL), 0, Watts, Phase, ChargeCost);
      	    printf("%02d:%02d:%02d %02d/%02d/%02d\r\n", (int)clock_Time.tm_hour, (int)clock_Time.tm_min, (int)clock_Time.tm_sec, (int)clock_Time.tm_mon, (int)clock_Time.tm_mday, (int)clock_Time.tm_year);
      	    LoopCount = 0;
    	}
#endif
        ISR_Flag &= ~ISRFLG_DMA_BIT;
        ADC12CTL0 |= ENC;
        TACCTL0 |= CCIE;
		clock_Time.tm_sec += 1;
        //    }

//    if (LoopCount == 70)
//    {

//        for (i = 0; i < Num_of_Results; i++)    {
//            sprintf(message,"%u \r\n ", buffer0[i]);               // Print out our Max value across Uart
//            PrintString(message);
//        }
//sprintf(message, "************************\r\n");
//PrintString(message);
    }
  }  // while(1)
}  // main()

/******************************* volt_zeroDetector ***************************/
long volt_zeroDetector(volatile unsigned int voltage[Num_of_Results])
{
    long PeriodTime = 0;
    int i, j = 0, state = 0, state_last = 0, diff1, diff2;

    Volts.Time[0] = 0;

    for (i = 1; i < Num_of_Results; i++)    {       // the first point always seems bad
        if (voltage[i] > Volts.mid)       state = 1;	// set state = 1, positive
        else if (voltage[i] < Volts.mid)  state = -1;	// set state = -1, negative
        else if (voltage[i] == Volts.mid)  state = 0;	// set state = 0, when equal
        if (i > 1)  {
            if (state == 0)                 Volts.Time[j++] = i;
            else if (state_last != state)   Volts.Time[j++] = i;	// look for state transitions, but don't know if prior point was closer
        }
        state_last = state;
        if (j == 3) break;
    }
	for (j = 0; j < 3; j++)  {
    	if (voltage[Volts.Time[j]-1] > Volts.mid) 	diff1 = voltage[Volts.Time[j]-1] - Volts.mid;
    	else 										diff1 = Volts.mid - voltage[Volts.Time[j-1]];
    	if (voltage[Volts.Time[j]] > Volts.mid) 	diff2 = voltage[Volts.Time[j]] - Volts.mid;
    	else 										diff2 = Volts.mid - voltage[Volts.Time[j]];
 		if (diff1 < diff2)  Volts.Time[j] -= 1;
	}
    PeriodTime = Volts.Time[2] - Volts.Time[0];
    if (PeriodTime < 800) {		// debugging bad results
        for (i = 0; i < Num_of_Results; i++)    printf("%u \r\n ", voltage[i]);               // Print out our Max value across Uart
        printf(message, "************************\r\n");
    }
    return PeriodTime;
}

/********************************* RefineCrossing ****************************
Description: Multi-step process to refine zero crossing.  Step 1: refine 
zero crossing voltage (Volts.mid).  Step 2
****************************************************************************/
long RefineCrossing(volatile unsigned int voltage[Num_of_Results])
{
	long value1 = 0, value2 = 0;
	int i, slope = 0, j = 0, min_value = -32767, max_value = 32767;
	int Crossing[4], value, midV, Scaling;

	// Use zero crossing values to refine peak, min, delta, and mid values
	i = Volts.Time[1] - Volts.Time[0];		// first max / min
	for (j = Volts.Time[i - 4]; j < Volts.Time[i + 4]; j++)  {
		value1 += voltage[j];		// find the delta voltage at 8 points around each zero crossing
	}
	i = Volts.Time[2] - Volts.Time[1];		// second max / min
	for (j = Volts.Time[i - 4]; j < Volts.Time[i + 4]; j++)  {
		value2 += voltage[j];		// find the delta voltage at 8 points around each zero crossing
	}
	if (value1 > value2) {  // not sure if value1 is Volts.peak or Volts.min
		Volts.peak = (int)value1;
		Volts.min = (int)value2;
		Volts.delta = value1 - value2;
	}
	else {
		Volts.peak = (int)value2;
		Volts.min = (int)value1;
		Volts.delta = value2 - value1;
	}
	Volts.peak >>= 3;			// Averaged 8 values
	Volts.min >>= 3;			// Averaged 8 values
	Volts.delta >>= 3;			// Averaged 8 values
	Volts.mid = (value1 + value2) >> 4;		// midpoint of 8 averaged values
	midV = (value1 + value2) >> 3;
	
	// Find the closest point on the voltage curve to the zero crossing using new Volts.mid
	j = Volts.Time[0];		// first zero crossing
	for (j = Volts.Time[i - 4]; j < Volts.Time[i + 4]; j++)  {
		value = (voltage[j] << 1) - midV;
		if (value == 0)  {
			Crossing[0] = (j << 5);
			break;
		}
		else if ((value > 0) && (value < max_value)) 	{
			max_value = value;
			Crossing[0] = (j << 5);
		}	
		else if ((value < 0) && (value > min_value)) 	{
			min_value = value;
			Crossing[0] = (j << 5);
		}	
	}

	
	// Volts.mid is a refined zero crossing voltage.  Slope is 24x actual slope
	// Algorithm minimizes the difference between measured voltages (voltage[j]) and
	// the slope-based voltages to get better approximate the zero crossing time.
	// If floats were used, the algorithm might be easier to follow
	slope >>= 1;	// slope is 12x actual
	j = Volts.Time[0];
	Crossing[0] = (Volts.peak - Volts.min);
	Crossing[1] = (voltage[j-2] << 1) - Crossing[0];  // Need to figure out if add slope to result or subtract result
	Crossing[2] = (voltage[j] << 1) - Crossing[0];		// Ideally, this value should be zero
	Crossing[3] = (voltage[j+2] << 1) - Crossing[0];
	if (Crossing[1] > 0)	{  // A.C. wave is crossing the zero-axis from + to -
		Scaling = (Crossing[2] - Crossing[1]) / (Crossing[3] - Crossing[1]);
	}
	else if (Crossing[3] > 0)	{  // A.C. wave is crossing the zero-axis from - to +
		Scaling = (Crossing[2] - Crossing[3]) / (Crossing[3] - Crossing[1]);
	}
	
	
	return value1;
}


/***************************************************************************

     Description: This function returns an rolling average value of the last
     eight input signal values
****************************************************************************/
long averagePeriod(long AvgPeriod, long iPeriod)
{
    long calcAvg = AvgPeriod;

    calcAvg -= (AvgPeriod >> AVG_INTERVAL);
    calcAvg += iPeriod;
    return calcAvg;
}


//***************************************************************************
int voltConv(volatile unsigned int voltage[Num_of_Results])
{
    int i = 0;

    Volts.peak = 0;
	Volts.min = 32767;

	for (i = 0; i < Num_of_Results; i++)  {
		if (Volts.peak < voltage[i]) Volts.peak = voltage[i];
		if (Volts.min > voltage[i]) Volts.min = voltage[i];
	}
	Volts.delta = Volts.peak - Volts.min;
	Volts.mid = (Volts.delta >> 1) + Volts.min;
	return Volts.delta;
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
