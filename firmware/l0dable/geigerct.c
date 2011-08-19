/*
 * geigerct.c
 *
 *
 *  Created on: 11.08.2011
 *      Author: Turbo J <turboj@web.de>
 *
 *  Implements simple Geiger Counter
 *  Counts rising edges on P3_0 = BUSINT
 *  so you can directly connect the Geiger board
 *  from http://mightyohm.com/blog/products/geiger-counter/
 *
 */
#include <sysinit.h>
#include <string.h>

#include "basic/basic.h"
#include "basic/config.h"

#include "lcd/render.h"
#include "lcd/print.h"
#include "filesystem/ff.h"
#include "funk/mesh.h"

#include "usetable.h"

// Liberated from ARM Cortex M3 CMSIS core_cm3.h
// The processor definition headers for R0ket are incomplete :-/

#define __I
#define __IO volatile

typedef struct {
	__I uint32_t CPUID; /*!< Offset: 0x000 (R/ )  CPU ID Base Register                                  */
	__IO uint32_t ICSR; /*!< Offset: 0x004 (R/W)  Interrupt Control State Register                      */
	__IO uint32_t VTOR; /*!< Offset: 0x008 (R/W)  Vector Table Offset Register                          */
	__IO uint32_t AIRCR; /*!< Offset: 0x00C (R/W)  Application Interrupt / Reset Control Register        */
	__IO uint32_t SCR; /*!< Offset: 0x010 (R/W)  System Control Register                               */
	__IO uint32_t CCR; /*!< Offset: 0x014 (R/W)  Configuration Control Register                        */
	__IO uint8_t SHP[12]; /*!< Offset: 0x018 (R/W)  System Handlers Priority Registers (4-7, 8-11, 12-15) */
	__IO uint32_t SHCSR; /*!< Offset: 0x024 (R/W)  System Handler Control and State Register             */
	__IO uint32_t CFSR; /*!< Offset: 0x028 (R/W)  Configurable Fault Status Register                    */
	__IO uint32_t HFSR; /*!< Offset: 0x02C (R/W)  Hard Fault Status Register                            */
	__IO uint32_t DFSR; /*!< Offset: 0x030 (R/W)  Debug Fault Status Register                           */
	__IO uint32_t MMFAR; /*!< Offset: 0x034 (R/W)  Mem Manage Address Register                           */
	__IO uint32_t BFAR; /*!< Offset: 0x038 (R/W)  Bus Fault Address Register                            */
	__IO uint32_t AFSR; /*!< Offset: 0x03C (R/W)  Auxiliary Fault Status Register                       */
	__I uint32_t PFR[2]; /*!< Offset: 0x040 (R/ )  Processor Feature Register                            */
	__I uint32_t DFR; /*!< Offset: 0x048 (R/ )  Debug Feature Register                                */
	__I uint32_t ADR; /*!< Offset: 0x04C (R/ )  Auxiliary Feature Register                            */
	__I uint32_t MMFR[4]; /*!< Offset: 0x050 (R/ )  Memory Model Feature Register                         */
	__I uint32_t ISAR[5]; /*!< Offset: 0x060 (R/ )  ISA Feature Register                                  */
} SCB_Type;

#define SCS_BASE            (0xE000E000UL)                            /*!< System Control Space Base Address */
#define SCB_BASE            (SCS_BASE +  0x0D00UL)                    /*!< System Control Block Base Address */
#define SCB                 ((SCB_Type *)           SCB_BASE)         /*!< SCB configuration struct          */

#define MIN_SAFE_VOLTAGE 3650

void (*orig_handler_extint3)(void);  // original EINT3 handler

uint32_t volatile IntCtr;
uint32_t VectorTableInRAM[73] __attribute__ ((aligned(1024))); // VTOR needs 1024 Byte alignment, see UM10375.PDF

uint32_t dataBuf[30]={123};
uint8_t dataBufIdx=1;


void ExtInt3_Handler();

// Remember: ram() must be the first function, place all other code AFTER
// because the Implementer seem not to know how to use section attributes

static uint8_t mainloop();

static void intro(int num);


void ram(void) {
	uint8_t button;
	uint32_t LEDs;

	intro(3);
	dataBufIdx=0;
	// populate my Vector table
	memcpy(VectorTableInRAM, 0, sizeof(VectorTableInRAM));
	orig_handler_extint3 = (void*) VectorTableInRAM[EINT3_IRQn + 16];
	VectorTableInRAM[EINT3_IRQn + 16] = (uint32_t) &ExtInt3_Handler;
	// HACK: use RAM vector table to implement own External IRQ handler
	SCB->VTOR = (uint32_t) &VectorTableInRAM[0];
	// TODO add DMB() here, as VTOR updates are NOT effective immediately
	//
	GPIO_GPIO3IEV |= 1;
	GPIO_GPIO3IE |= 1;
	GPIO_GPIO3DIR &= ~1;
	GPIO_GPIO3IS &= ~1;
	GPIO_GPIO0DATA &= ~1;
	IOCON_PIO3_0 = (1 << 3) | (1 << 5); // Pull DOWN not Up, Hyst on
	NVIC_EnableIRQ(EINT3_IRQn);
	IntCtr = 0;
	LEDs = 0;
	mainloop();

	GPIO_GPIO3IE &= ~1; // disable GPIO IRQ
	NVIC_DisableIRQ(EINT3_IRQn);
	// restore VTOR
	SCB->VTOR = 0;
	//TODO DMB(); Cortex Manual suggests DMB after setting VTOR
	// not really needed in this case
}

void ExtInt3_Handler() {
	if (GPIO_GPIO3RIS & 0x01) {
		GPIO_GPIO3IC |= (0x01); // ACK BUSINT


		//GPIO_GPIO0DATA|=(1<<11);
		IOCON_PIO1_11 = 0;
		GPIO_GPIO1DATA |= (1 << 7);
		GPIO_GPIO1DATA |= (1 << 11);
		IntCtr++;
	} else {
		orig_handler_extint3();
	}
}

/* The datasheet for the SBM-20 tube says :
 *  Ra226: 29cps => 1mR/h = 10�Sv/h
 *  Co60:  22cps => 1mR/h = 10�Sv/h
 *
 *  //2.9 * 60 cpm = 1 �Sv/h
 */
static uint32_t nanoSievertPerH(uint32_t cpm) {

	return ((1000*cpm ) /(29*6) );

}



static void intro(int num){
  FIL file;
  int res;
  UINT readbytes=RESX*RESY_B;
  res=f_open(&file,"ranim.lcd",FA_OPEN_EXISTING|FA_READ);
  if (res) return;
  do {
	lcdFill(0x55);
	res = f_read(&file, (char *)lcdBuffer, RESX*RESY_B, &readbytes);
	if(res)
	return;
	if(readbytes<RESX*RESY_B) {
				f_lseek(&file,0);
				continue;
			};
	lcdRefresh();

	delayms(50);

  } while (--num);

}

const char * const dataFileName= "geiger.csv";

static void OpenDataFile(FIL * datafile){

	f_open(datafile,dataFileName,FA_WRITE|FA_OPEN_ALWAYS);
	f_lseek(datafile,datafile->fsize);

}

static void writeDataHeader(FIL * datafile) {
	UINT dummy;
	f_write(datafile,"-----\n\r",7,&dummy);
}

static void writeDataToFile(FIL* file){
	//char buf[16];
	for (int i=0; i<dataBufIdx; i++ ) {
		UINT nBytes=0;
		char *c=IntToStr(dataBuf[i],7,0);

	    f_write(file, c,strlen(c),&nBytes );
	    f_write(file,"\n\r",2,&nBytes);
	    if (GetVoltage()<MIN_SAFE_VOLTAGE) {
	    	f_close(file); // update FAT + Dir entry
	    	OpenDataFile(file);
	    }

	}

}



static uint8_t mainloop() {
	uint32_t ioconbak = IOCON_PIO1_11;
	UINT perMin;
	FIL datafile;
	OpenDataFile(&datafile);
	writeDataHeader(&datafile);
	uint32_t volatile oldCount=IntCtr;
	perMin=0; // counts in last 60 s
	uint32_t minuteTime=_timectr;
	uint32_t startTime=minuteTime;
	uint8_t button;
		IOCON_PIO1_11 = 0;
		while (1) {
			//GPIO_GPIO0DATA&=~(1<<11);
			IOCON_PIO1_11 = ioconbak;
			GPIO_GPIO1DATA &= ~(1 << 7);
			GPIO_GPIO1DATA &= ~(1 << 11);
			lcdClear();

			lcdPrintln("   Geiger");
			lcdPrintln("   Counter");
			// ####
			for (int i=0; i< (14*( _timectr-minuteTime))/(60*100);i++) {
				lcdPrint("#");
			}

			lcdPrintln("");
			lcdPrintln("Counts:");
			lcdPrint(" ");
			lcdPrintInt(IntCtr);
			lcdPrint(" in ");
			lcdPrintInt((_timectr-startTime)/100);
			lcdPrintln("s");
			lcdPrint(" ");
			lcdPrintInt(  perMin);
			lcdPrintln(" cpm");
			{

				uint32_t equivalent=nanoSievertPerH(perMin);
				lcdPrint(" ");
				lcdPrintInt(equivalent/1000);
				lcdPrint(".");
				lcdPrintInt((equivalent%1000) /100);
				lcdPrintInt((equivalent%100) /10);
				lcdPrintInt((equivalent%10));
				lcdPrintln(" uSv/h");

			}
			if (GetVoltage()<MIN_SAFE_VOLTAGE) {
				if (GetVoltage()<3550) {
					lcdPrintln("Battery CRIT!");
				} else
				lcdPrintln("Battery low");
			}
			// remember: We have a 10ms Timer counter
			if ((minuteTime+60 *100 ) < _timectr) {
				// dumb algo: Just use last 60 seconds count
				perMin=IntCtr-oldCount;
				minuteTime=_timectr;
				oldCount=IntCtr;
				dataBuf[dataBufIdx++]=perMin;
				if ((dataBufIdx >=30)||(GetVoltage()<MIN_SAFE_VOLTAGE)) {
					writeDataToFile(&datafile);
					dataBufIdx=0;
				}
			}
			lcdRefresh();
			delayms(42);
			button = getInputRaw();

			if (button != BTN_NONE) {
				delayms(23);// debounce and wait till user release button
				while (getInputRaw()!=BTN_NONE) delayms(23);
				break;
			}
		}

		if (dataBufIdx) writeDataToFile(&datafile);
		f_close(&datafile);
		IOCON_PIO1_11 = ioconbak;
		return button;

}


