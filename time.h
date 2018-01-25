/* Copyright © 1995-2007 Freescale Corporation.  All rights reserved.
 *
 * $Date: 2007/01/29 18:12:27 $
 * $Revision: 1.43 $
 */

#ifndef _TIME_H
#define _TIME_H

//#include "PE_Types.h"
static int adjust(int * x, int y, int * z);
int leap_year(int year);
static int leap_days(int year, int mon);

typedef unsigned long time_t;

typedef struct {
	int		tm_sec;
	int		tm_min;
	int		tm_hour;
	int		tm_mday;
	int		tm_mon;
	int		tm_year;
	int		tm_wday;
	int		tm_yday;
	int		tm_isdst;
} tm_t;

unsigned long tm2time(tm_t *);
void time2tm(time_t inTime, tm_t *);

#endif
