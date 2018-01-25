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

int  Assign(char option, long value, int Count);
int fputc(int _c, register FILE * _fp);
int fputs(const char *_ptr, register FILE *_fp);
void InitSystem(void);
void InitTimers(void);
void InitUart(void);
int  ParseString(char * theString, int index);
void PrintDate(int month, int day, int year);
void PrintDec(long value);
void PrintDec1(long value);
void PrintTime(int hour, int min, int sec);
void PrintString(char * theString);
void Record(void);
void SerialReply(char option);
int  ServiceSerial(void);
int  Uart0TxChar(uint8 c);
int  Uart1TxChar(uint8 c);

// Global Variables
int TxCharCount = 0, TxReadIndex = 0, TxWriteIndex = 0;
int RxWriteIndex = 0, RxReadIndex = 0;
char TxBuffer[SER_BUFFER_SIZE], RxBuffer[SER_BUFFER_SIZE];
int ISR_Flag = 0;
tm_t clock_Time = {0, 0, 11, 7, 12, 2017, 1};	//(sec, min, hr, day, mon, year,0);

volatile unsigned int results[Num_of_Results], convResults[Num_of_Results];
double frequency[Num_of_Results]; // global frequency
int offset[6]; // global offset
char message[SER_BUFFER_SIZE];

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

  long Period = 5333333, PeriodAvg = 5333333, ChargeCost = 0;
  int battCurrent = 0, battSOC = 20, TotWsec = 0;
  int iVolts = 115, aVolts = 115, Watts = 3000, Phase  = 0;

  InitSystem();
  InitUart();
  InitTimers();					// Enable tic timer

  while (1) {
    __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupts

    if (ISR_Flag & ISRFLG_USB_BIT)  {
      ISR_Flag &= ~ISRFLG_USB_BIT;
      ServiceSerial();
    }
    if (ISR_Flag & ISRFLG_TWOHZ) {
      LoopCount++;
      ISR_Flag &= ~ISRFLG_TWOHZ;
    }

    if (ISR_Flag & ISRFLG_DMA_BIT)  {
    	DMA0CTL &= ~DMAEN;
    	TBCTL = 0;
//    	Period = singlePeriod(buffer0);
//		PeriodAvg = averagePeriod(Period,PeriodAvg);
		ISR_Flag &= ~ISRFLG_DMA_BIT;
    }
    else if (LoopCount == 16) {
        P1OUT ^= BIT0;
      	LoopCount = 0;
      	tm2time(&clock_Time);
      	strcpy(&message[0], "X= ");
        Uart0TxChar('X');

		PrintDec1(Period);  	// concatenates Period and ", " to the message string
 		PrintDec1(PeriodAvg);  	// concatenates PeriodAvg and ", " to the message string
    	PrintDec1(battCurrent); // concatenates battCurrent and ", " to the message string
    	PrintDec1(battSOC);		// concatenates battSOC and ", " to the message string
    	PrintDec1(TotWsec);  	// concatenates TotWsec and ", " to the message string
    	PrintDec(iVolts);		// concatenates iVolts and ", " to the message string
        strcat(message, ".");
    	PrintDec1(0);			// concatenates 0 and ", " to the message string
    	PrintDec(aVolts);		// concatenates aVolts and ", " to the message string
        strcat(message, ".");
    	PrintDec1(0);			// concatenates 0 and ", " to the message string
    	PrintDec1(Watts);		// concatenates Watts and ", " to the message string
    	PrintDec1(Phase);		// concatenates Phase and ", " to the message string
		PrintDec1(ChargeCost);	// concatenates ChargeCost and ", " to the message string
        PrintTime((int)clock_Time.tm_hour, (int)clock_Time.tm_min, (int)clock_Time.tm_sec);
    	PrintDate((int)clock_Time.tm_mon, (int)clock_Time.tm_mday, (int)clock_Time.tm_year);
    	strcat(message, " ");
//    	PrintDec1(port3data);
    	PrintDec1(GpsRxIndex);
        strcat(message, "\r\n");

    	PrintString(message);	// Output the message screen out the serial port
//    	PrintString("\r\n");
      	Period -= 100;
      	if (Period < 5332445) Period = 5333333;
    	clock_Time.tm_sec += 1;
    	Watts +=100;
    	if (Watts > 3500) Watts = 2500;
    	Phase +=5;
    	if (Phase > 180) Phase = -180;
    	iVolts += 1;
    	if (iVolts > 116) iVolts = 114;
    	aVolts += 1;
    	if (aVolts > 116) aVolts = 114;
    }
  }  // while(1)
}  // main()

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

/***************************************************************************
 	 	 	 	 	 freqCalc
 	 Description: This function takes the digital voltage values read
 	 	 	 	  into the chip and converts it into frequency based
 	 	 	 	  off of the sample rate (78125 Hz) of the ADC.
****************************************************************************/
void freqCalc(volatile unsigned int voltage[Num_of_Results], int *offset, double frequency[Num_of_Results])
{
	int count = 0, i = 0, cur = 0, prev = 0, next = 0, start = 0, zeros[Num_of_Results], cur2 = 0, prev2 = 0;
	long double period = 0.0;
	long double tick = 0.0001004016;

	if (*offset != 0)
	{
		zeros[count] = offset[0];
		count++;
		zeros[count] = offset[1];
		count++;
		frequency[0] = frequency[Num_of_Results - 1];
		start = 1;
	}

	else
	{
		frequency[0] = 60.0;
		frequency[1] = 60.0;
		frequency[2] = 60.0;
		frequency[3] = 60.0;
		start = 4;
	}

	for (i = start; i < Num_of_Results; i++)
	{
		if (i == 0)
		{
			cur = offset[4];
			prev = offset[2];
		}

		else if (i == 1)
		{
			cur = offset[5];
			prev = offset[3];
		}

		else if (i == 2)
		{
			cur = voltage[i - 2];
			prev = offset[4];
		}

		else if (i == 3)
		{
			cur = voltage[i - 2];
			prev = offset[5];
		}

		else
		{
			cur = voltage[i - 2];
			prev = voltage[i - 4];
		}

		next = voltage[i];

		if ((cur < prev) && (next > cur) && (cur < 3000))
		{
			zeros[count] = i;
			if (count < 1)
			{
				count++;
			}

			else if (abs(zeros[count] - zeros[count -1]) > 40)
			{
				count++;
			}

			if (count > 2)
			{
				cur2 = zeros[count-1];
				prev2 = zeros[count-2];
				period = tick*2*(cur2-prev2);
				frequency[i] =  (1/period);

			}

			else
			{
				frequency[i] = 60.0;
			}
		}

		else
		{
			frequency[i] = frequency[i-1];
		}
	}

	offset[0] = zeros[count - 2] - Num_of_Results;
	offset[1] = zeros[count - 1] - Num_of_Results;
	offset[2] = voltage[Num_of_Results - 4];
	offset[3] = voltage[Num_of_Results - 3];
	offset[4] = voltage[Num_of_Results - 2];
	offset[5] = voltage[Num_of_Results - 1];
}

double voltConv(volatile unsigned int voltage[Num_of_Results])
{
	int peak = 0, i = 0;
	double vrms = 0.0;

	for (i = 0; i < Num_of_Results; i++)
	{
		if (peak < voltage[i]) peak = voltage[i];
	}

	vrms = 0.000732*peak*sqrt(2);

	return vrms;
}

#ifdef UART_PRINTF
// ***************************************************************************
int fputc(int _c, register FILE *_fp)
{
    while (!(UC1IFG & UCA1TXIFG));
    UCA1TXBUF = (unsigned char) _c;
    return (unsigned char) _c;
}

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

  P3SEL |= 0x30;                             // P3.4,5 = USCI_A1 TXD/RXD
  UCA0CTL1 |= UCSSEL_2;                     // SMCLK
  UCA0BR0 = 82;                              // 8MHz 4800
  UCA0BR1 = 6;                              // 8MHz 4800
  UCA0MCTL = UCBRS2 | UCBRS1;               // Modulation UCBRSx = 6 for 8MHz
  UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
  UC0IE |= UCA0RXIE;                          // Enable USCI_A0 RX interrupt
}


/*****************************************************************************
    InitSystem function - set constants & I/O
*****************************************************************************/
void InitSystem(void) {
  unsigned int i;

  WDTCTL = WDTPW + WDTHOLD;     // Stop watchdog timer
  // set first functionality to all ports
  BCSCTL1 = CALBC1_8MHZ;                    // Set DCO to 1MHz , this should be 8 Mhz and the clock is divided to 1Mhz in inittimer function
  DCOCTL = CALDCO_8MHZ;
  BCSCTL2 = 0;                  // SMCLK = MCLK = DCOCLK / 1
  for (i = 0xfffe; i > 0; i--);             // Delay for XTAL stabilization

  P1SEL = 0x00;
  P2SEL = 0x00;     // P2SEL must have BIT6/7 HIGH to enable 32kHz crystal, ACLK Active, P2.3 - P2.4 TA
  P3SEL = 0x00;
  P4SEL = 0x00;
  P5SEL = 0x00;
  P6SEL = 0x01;
  P7SEL = 0x00;
  P8SEL = 0x00;

  //set pin direction and initialize outputs
  P1DIR |= 0x09;                            // P1.3 output
  TACCTL0 = 0x00;                // TACCR1 toggle, interrupt enabled
  TACCTL1 = 0x00;
  TACCTL2 = 0x00;
  TBCCTL0 = 0x00;
  TBCCTL1 = 0x00;
  TBCCTL2 = 0x00;
  ADC12CTL0 = ADC12ON + SHT0_8 + MSC;   // Turn ON ADC12, set sampling time
  ADC12CTL1 = SHP + CONSEQ_2;           // Use sampling timer, set mode
//  ADC12IE = BIT0;
  ADC12CTL0 |= ENC;                     // Enable conversions
  ADC12CTL0 |= ADC12SC;                 // Start conversions
  DMA0SZ = Num_of_Results;                               // Size in words
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
  TACTL = 0;
  TACTL = TASSEL_2 + TACLR + ID_3;      // SMCLK/8, clear TAR
  TACCTL0 = CCIE;                       // CCR0 interrupt enabled
  TACCR0 = 50000;
  TACTL |= MC1;                         // Start Timer_a in upmode
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
							PrintDec1
Description: Prints a long value as a decimal.
****************************************************************************/
void PrintDec1(long value)
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
    strcat(message, ", ");
}

/****************************************************************************
							PrintTime
Description: Prints a formatted time.
****************************************************************************/
void PrintTime(int hour, int min, int sec)
{
	char digit[9];

	digit[0] = '0' + (unsigned char) (hour / 10);
	digit[1] = '0' + (unsigned char) (hour % 10);
    digit[2] = ':';
	digit[3] = '0' + (unsigned char) (min / 10);
	digit[4] = '0' + (unsigned char) (min % 10);
    digit[5] = ':';
	digit[6] = '0' + (unsigned char) (sec / 10);
    digit[7] = '0' + (unsigned char) (sec % 10);
    digit[8] = NULL;
    strcat(message, digit);			//    PrintString(digit);		//
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

// ********* parse_line **************************************** //
static void parse_line()
{
    switch (minmea_sentence_id(line, false)) {
        case MINMEA_SENTENCE_GGA: {
            struct minmea_sentence_gga frame;
            if (minmea_parse_gga(&frame, line)) {
                if(frame.fix_quality > 0)  {
                  lon = frame.longitude.value;
                  lat = frame.latitude.value;
                  altitude = (long)minmea_tofloat(&frame.altitude);
                  height = (long)minmea_tofloat(&frame.height);
                  if(frame.fix_quality > 0){
                    printf("Fix: %d Sat: %d Time: %2d:%2d:%2d Long: %ld Lat: %ld Height: %ld Altitude: %ld Long: %f\r\n",
                           frame.fix_quality, frame.satellites_tracked,
                           frame.time.hours, frame.time.minutes, frame.time.seconds,
                           lon, lat, height, altitude,
                           minmea_tocoord(&frame.longitude));
                  }
//                  ctimer_stop(&nofix_timer);
//                  shutDown(NULL);
//                  notify_ready(NULL);
                }
                else printf("No fix - reset host\r\n");
            }
            break;
        }
        case MINMEA_SENTENCE_GLL:
            break;
        case MINMEA_SENTENCE_GST:
            break;
        case MINMEA_SENTENCE_GSV:
            break;
        case MINMEA_SENTENCE_GSA:
            break;
        case MINMEA_SENTENCE_RMC:
            break;
        case MINMEA_SENTENCE_VTG:
            break;
        case MINMEA_INVALID:
            break;
        case MINMEA_UNKNOWN:
            break;
        default:
            PrintString("/r/nNo matching MINMEA case\r\n");
            break;
    }
}


// Record ADC12 A10 channel data and store in RAM using DMA0
void Record(void)
{
  // setup modules
  volatile unsigned int i;
  ADC12MCTL0 = 0x01A;                       // Vref+, Channel A10
  ADC12IFG = 0x00;
  ADC12CTL1 = SHS_3 + CONSEQ_2;             // S&H TB.OUT1, rep. single chan
  ADC12CTL0 = REF2_5V + REFON + ADC12ON + ENC;    // VRef ADC12 on, enabled
  for (i = 0xFFF; i > 0; i--);              // Time VRef to settle

  TBCCR0 = 100;                             // Init TBCCR0 w/ sample prd
  TBCCR1 = 100 - 30;                        // Trigger for ADC12 SC
  TBCCTL1 = OUTMOD_7;                       // Reset OUT1 on EQU1, set on EQU0

  DMACTL0 = DMA1TSEL_11 + DMA0TSEL_6;       // DMA1=MPY Ready, DMA0=ADC12IFGx

  DMA0SA = (void (*)())&ADC12MEM0;          // Src address = ADC12 module
  DMA0DA = (void (*)())&OP2;                // Dst address = multiplier
  DMA0SZ = 1;                               // Size in words
  DMA0CTL = DMADT_4 + DMAEN;                // Sng rpt, config

  DMA1SA = (void (*)())&RESHI;              // Src address = multiplier
  DMA1DA = (void (*)())0x220;               // Dst address = RAM memory
  DMA1SZ = 0x20;                            // Size in bytes
  DMA1CTL = DMADSTINCR_3 + DMASBDB + DMAIE + DMAEN;   // Sng, config

  MPY = 0x1000;                             // MPY first operand

  // start recording and enter LPM0
  P1OUT |= 0x01;                            // Set P1.0 (LED On)
  TBCTL = TBSSEL_2 + MC_1 + TBCLR;          // SMCLK, clear TBR, up mode

  _BIS_SR(LPM0_bits + GIE);                 // Enter LPM0 w/ interrups

  // power-down MSP430 modules
  ADC12CTL1 &= ~CONSEQ_2;                   // Stop conversion immediately
  ADC12CTL0 &= ~ENC;                        // Disable ADC12 conversion
  ADC12CTL0 = 0;                            // Switch off ADC12 & ref voltage
  TBCTL = 0;                                // Disable Timer_B
  P1OUT &= ~0x01;                           // Clear P1.0 (LED Off)
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

// Load UART TxBuffer and enable TX interrupt to send character to the GPS
int Uart0TxChar(uint8 c)
{
  IE2 &= ~UCA0TXIE;
  while (!(IFG2 & UCA0TXIFG));
  UCA0TXBUF = c;
//  IE2 |= UCA0TXIE;  // Add ISR to support this interrupt
  return (int)0;
}

// Load TxBuffer and enable TX interrupt to send character out the USB Serial Port
int Uart1TxChar(uint8 c)
{
  UC1IE &= ~UCA1TXIE;
  TxBuffer[TxWriteIndex++] = c;
  TxWriteIndex &= (SER_BUFFER_SIZE - 1);
  TxCharCount++;
  UC1IE |= UCA1TXIE;
  return (int)TxWriteIndex;
}

//***************************************************************************
// Timer A1 interrupt service routine
#pragma vector=TIMERA0_VECTOR
__interrupt void Timer_A (void)
{
  P1OUT ^= 0x08;                            // Toggle P1.3
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
  if (TxCharCount > 0)	{
    UCA1TXBUF = TxBuffer[TxReadIndex++];
    TxReadIndex &= (SER_BUFFER_SIZE -1);
    TxCharCount--;
  }
  if (TxCharCount == 0) UC1IE &= ~UCA1TXIE;
  __bic_SR_register_on_exit(LPM4_bits + GIE);     // Exit LPMx mode
}

// Port 1 interrupt service routine *******************************
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
  P1IFG &= ~0x01;                          // P1.0 IFG cleared
  ISR_Flag |= BIT1;
  __bic_SR_register_on_exit(LPM4_bits + GIE);     // Exit LPMx mode
}


// DMA interrupt service routine
#pragma vector = DMA_VECTOR
__interrupt void DMA_ISR(void)
{
  DMA1CTL &= ~DMAIFG;                       // Clear DMA1 interrupt flag
  ISR_Flag |= ISRFLG_DMA_BIT;
  _BIC_SR_IRQ(LPM0_bits);                   // Exit LPM0 on reti
}


#pragma vector=ADC12_VECTOR
__interrupt void ADC12ISR (void)
{
  static unsigned int index = 0;

  results[index] = ADC12MEM0;               // Move results
  index = (index+1)%Num_of_Results;         // Increment results index, modulo;
/*
  if (index == 0)
  {
    freqCalc(results, offset, frequency);
   // vrms = voltConv(results);					// caclulate vrms and then output (serial port?) also need to output frequency to serial port
    lcount++;
    //TxCharCount = 1;
    // TxBuffer[0] = 0xa0;  //serial port code

    //index = 0;

    // DMA code
    __data16_write_addr((unsigned short) &DMA0SA,(unsigned long) results); //(source block address)
    __data16_write_addr((unsigned short) &DMA0DA,(unsigned long) "destination"); //(destination single address)

    DMA0SZ = 0x0010;                          // Block size
  	DMA0CTL = DMADT_5 + DMASRCINCR_3 + DMADSTINCR_3; // Rpt, inc
  	DMA0CTL |= DMAEN;                         // Enable DMA0
 */

/*
  }
  if (lcount == 5) //
  {
	  if (frequency[205] >= 60.07)	P1OUT |= 0x04;			// Set P1.0 LED on
	  else						  	P1OUT &= ~0x01;			// Clear P1.0 LED off
	  lcount = 0;
  }
  */
}
