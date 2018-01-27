/* Copyright © 1995-2007 Freescale Corporation.  All rights reserved.
 * $Date: 2007/01/29 18:11:40 $
 * $Revision: 1.90 $
 */

#include <string.h>
#include "time.h"
#include "Gfa.h"

#define ULONG_MAX			0xFFFFFFFF
#define seconds_per_minute		(60L)
#define seconds_per_hour		(60L * seconds_per_minute)
#define seconds_per_day			(24L * seconds_per_hour)
#define seconds_1900_to_1970	(((365 * 70UL) + 17) * 24 * 60 * 60)		/*- mm 000127 -*/

const unsigned int Months[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

const short month_to_days[2][13] = {
	{0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334, 365 },
	{0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335, 366 }};

// leap_year - return nonzero if year is a leap year, zero otherwise
// (year 0 = 1900)
// The date system is based on the Gregorian calendar. In this calendar,
// a normal year consists of 365 days. Because the actual length of a
// sidereal year (the time required for the Earth to revolve once about
// the Sun) is actually 365.25635 days, a "leap year" of 366 days is used
// once every four years to eliminate the error caused by three normal
//(but short) years. Any year that is evenly divisible by 4 is a leap year.
// To eliminate a remaining small error, the Gregorian calendar stipulates
// that a year that is evenly divisible by 100 (for example, 1900) is a leap
// year only if it is also evenly divisible by 400.

int leap_year(int year)
{
	return (((year % 4) == 0) && (((year % 100) != 0) || ((year % 400) == 0)));
}

/****************************************************************************
*  leap_days - return the number of leap days between 1900 (year 0) and the
*  given year and month. year can be negative, month cannot.
****************************************************************************/
static int leap_days(int year, int mon)
{
    int		q, n;

    n = year / 4;		// 1 leap day every four years
    q = year / 100;		// except at the turn of the century
    n -= q;
    if (year < 100)	{		// not counting the turn of the millenium
      q = (year + 899) / 1000;		/*- cc 010510 -*/
      n += q;
    }
    else	{
      q = (year - 100) / 1000;	/*- cc 010510 -*/
      n += q + 1;
    }

    if (leap_year(year))
        if (year < 0)	{
          if (mon > 1)	++n;
        }
        else if (mon <= 1)  --n;
    return(n);
}

/****************************************************************************
*	adjust - force x to be a modulo y number, add overflow to z
****************************************************************************/
static int adjust(int * x, int y, int * z)
{
  int q;

  q = *x / y; /*- cc 010510 -*/
  if (q)	{
    *x = *x % y;
    (*z)++;
  }
  return(1);
}

/****************************************************************************
*	time2tm - convert seconds since midnight, 1/1/1970 to broken-down time
****************************************************************************/
void time2tm(time_t inTime, tm_t *tm)		/*- mm 000127 -*/
{
    unsigned long	months, days, seconds;
    int				years;
    unsigned int is_leap_year;

    /* Since time_t is relative to 1970 rather than 1900,
     * This must be of type unsigned long rather than a signed
     * time_t to prevent overflow */
    unsigned long time = inTime; // + seconds_1900_to_1970;		/*- mm 000127 -*/

    if (!tm)	return;

/*	tm->tm_isdst = __isdst(); */

    days    = time / seconds_per_day;
    seconds = time % seconds_per_day;

    tm->tm_wday = (int)((days + 1) % 7);	// January 1, 1900 was a Monday
    years = 2000;
    for (;;)	{
            uint32 days_this_year = (uint32)(leap_year(years) ? 366 : 365);
            if (days < days_this_year)	break;
            days  -= days_this_year;
            years += 1;
    }
    tm->tm_year = years;
    tm->tm_yday = (int)days;
    months = 1;
    is_leap_year = leap_year(years);
    for (;;)	{
            unsigned long days_thru_this_month = month_to_days[is_leap_year][months];
            if (days < days_thru_this_month)	{
                    days -= month_to_days[is_leap_year][months - 1];
                    break;
            }
            ++months;
    }

    tm->tm_mon  = (int)months;
    tm->tm_mday = (int)days + 1;
    tm->tm_hour = (int)(seconds / seconds_per_hour);
    seconds %= seconds_per_hour;
    tm->tm_min = (int)(seconds / seconds_per_minute);
    tm->tm_sec = (int)(seconds % seconds_per_minute);
}


/*****************************************************************************
*	tm2time - convert broken-down time hr/min/sec/da/mo/yr to seconds since
*	midnight, 1/1/1970. return zero if broken-down time can't be represented;
*	otherwise, convert seconds back into broken-down time and return nonzero.
*
*	Note:	Each field in broken-down time is forced into the normal range for
*	that field, with overflow being added to next field up through mday
*	(day of month). mday is allowed to remain out of range. mon is forced into
*	its normal range with overflow being added to year. year and mon are
*	converted to days since 1/1/1970 with mday and leap days between 1/1/1970
*	and year and mon added.  If the result is negative, zero is returned.
*	Otherwise, it is converted to seconds and added to hour, min and sec
*	converted to seconds to yield the final result. Overflow checking is
*	performed where needed and if overflow occurs, zero is returned; nonzero
*	otherwise.
*****************************************************************************/
uint32 tm2time(tm_t *tm)
{
	#define REFERENCE_YEAR 2000
	long long days, days1;
	uint32 seconds;

	if (!tm)	return(0);
	adjust(&tm->tm_sec,  60, &tm->tm_min);		// put sec  in range 0-59
	adjust(&tm->tm_min,  60, &tm->tm_hour);		// put min  in range 0-59
	adjust(&tm->tm_hour, 24, &tm->tm_mday);		// put hour in range 0-23
	adjust(&tm->tm_mon,  13, &tm->tm_year);		// put mon  in range 0-12
	days = tm->tm_year;
	days *= 365;								// convert year to days
	days += leap_days(tm->tm_year, tm->tm_mon);	// add leap days
	days += month_to_days[0][tm->tm_mon - 1];	// add days to month
	days += tm->tm_mday;						// add days in month
	// Now make relative to 1/1/2000 @ midnight
	days1 = REFERENCE_YEAR * 365;
	days1 += leap_days(REFERENCE_YEAR, 1);		// add leap days
	days -=days1;

	if (leap_year(tm->tm_year) == 0)
		tm->tm_yday = month_to_days[0][tm->tm_mon - 1] + tm->tm_mday;
	else
		tm->tm_yday = month_to_days[1][tm->tm_mon - 1] + tm->tm_mday;
	tm->tm_wday = (int)((days + 4) % 7);		// 1/1/1970 is a Thursday

	if (days > ULONG_MAX / seconds_per_day)	return(0);	// make sure we're in range
	seconds = (uint32)(days * seconds_per_day);		// convert days to seconds
	seconds += (tm->tm_hour * seconds_per_hour);
	seconds += (tm->tm_min  * seconds_per_minute) + (uint32)tm->tm_sec;
	return(seconds);
}

