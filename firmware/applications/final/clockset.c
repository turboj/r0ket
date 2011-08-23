/*
 * clockset.c
 *
 *  Created on: 23.08.2011
 *      Author: turboj
 */
#include <sysinit.h>
#include <string.h>
#include <time.h>

#include "basic/basic.h"
#include "basic/byteorder.h"
#include "basic/config.h"
#include "basic/simpletime.h"

#include "lcd/lcd.h"
#include "lcd/print.h"

#include "filesystem/ff.h"
#include "filesystem/select.h"
#include "filesystem/execute.h"

#include <string.h>

static int days_per_month (int year, int month)
{
  if (month == 1)
    {
      if (LEAPYEAR(year))
        return 29;
      return 28;
    }
  else if (month > 6)
    month--;
  if ((month % 2))
    return 30;
  else
    return 31;
}

static uint64_t uatoi(char * buf) {
	uint64_t i=0;

	while( (*buf >='0')&&(*buf<='9')) {
		i=10*i+(*buf-'0');
		buf++;
	}

	return i;
}

//# MENU clock
void doClockSet(void){
char buffer[12];
struct tm * mytm=mygmtime(getSeconds());

	uint32_t big1;

	uint32_t offset=0;

	strcpy(buffer, IntToStr(mytm->tm_year+YEAR0,4,F_LONG|F_ZEROS));
	strcpy(buffer+4, IntToStr(mytm->tm_mon+1,2,F_LONG|F_ZEROS));
	strcpy(buffer+6, IntToStr(mytm->tm_mday,2,F_LONG|F_ZEROS));
	input("Date YYYYMMDD:", (buffer), '0', '9', 9);
	big1=uatoi(buffer);
	{
		lcdClear();
		uint32_t Year=big1/10000;
		uint32_t Month=(big1/100) %100;
		uint32_t day=big1%100;

		Month--;
		while(Month>0){ offset+=days_per_month(Year,--Month);}
		while(Year>EPOCH_YR) {
			Year--;  //beware of MACRO !!
			offset+= YEARSIZE(Year);
		}
		offset+=(day-1);

		offset=offset* (unsigned long)SECS_DAY ;
	}
	strcpy(buffer, IntToStr(mytm->tm_hour,2,F_LONG|F_ZEROS));
	strcpy(buffer+2, IntToStr(mytm->tm_min,2,F_LONG|F_ZEROS));
	strcpy(buffer+4, IntToStr(mytm->tm_sec,2,F_LONG|F_ZEROS));
	buffer[6]=0;
	buffer[7]=0;
	buffer[8]=0;
	input("Time HHMMSS:", buffer,'0', '9', 7);
	big1=uatoi(buffer);
	{
		uint32_t Hours=big1/10000;
		uint32_t minutes=(big1/100)%100;
		uint32_t seconds=big1%100;

		offset+=(Hours*3600UL)+(minutes*60UL)+seconds;

		offset-=getSeconds() - _timet;
		_timet=offset;


	}

	getInputWait();

}
