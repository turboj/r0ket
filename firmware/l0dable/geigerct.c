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

#include "../core/rom_drivers.h"
#include "../usb/usb.h"
//#include "../usb/usbconfig.h"
#include "../usb/usbhid.h"

#include "usetable.h"

#define USB_VENDOR_ID  0x4242
#define USB_PROD_ID 0x1977
#define USB_DEVICE 0x101

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
#define CRIT_VOLTAGE 3550

#define LED_ON	GPIO_GPIO1DATA |= (1 << 7)
#define LED_OFF GPIO_GPIO1DATA &= ~(1 << 7)

uint32_t VectorTableInRAM[73]  __attribute__ ((aligned(512)))={1234}; // VTOR needs 1024 Byte alignment, see UM10375.PDF
																	  // set to 512 to resolve a Linker Bug

void (*orig_handler_extint3)(void)__attribute__ ((aligned))=(void*)0x000123;  // original EINT3 handler

uint32_t volatile IntCtr __attribute__ ((aligned))=1;

void ExtInt3_Handler();

// Remember: ram() must be the first function, place all other code AFTER
// because the Implementer seem not to know how to use section attributes

static uint8_t mainloop();

static void intro(int num);
//static void transmitGeigerMeshVal();
static void getGeigerMeshVal();



void ram(void) {
	//uint8_t button;
	uint32_t LEDs;
	intro(3);

	// populate my Vector table
	memcpy(VectorTableInRAM, 0, sizeof(VectorTableInRAM));
	orig_handler_extint3 = (void*) VectorTableInRAM[EINT3_IRQn + 16];
	VectorTableInRAM[EINT3_IRQn + 16] = (uint32_t) &ExtInt3_Handler;
	// HACK: use RAM vector table to implement own External IRQ handler
	SCB->VTOR = (uint32_t) &VectorTableInRAM[0];
	//  add DMB() here, as VTOR updates are NOT effective immediately
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
}

void ExtInt3_Handler() {
	if (GPIO_GPIO3RIS & 0x01) {
		GPIO_GPIO3IC |= (0x01); // ACK BUSINT



//		IOCON_PIO1_11 = 0;
		LED_ON;
//		GPIO_GPIO1DATA |= (1 << 11);
		IntCtr++;
	} else {
		orig_handler_extint3();
	}
}

/**
 *  The datasheet for the SBM-20 tube says :
 *  Ra226: 29cps => 1mR/h = 10µSv/h
 *  Co60:  22cps => 1mR/h = 10µSv/h
 *
 *  2.9 * 60 cpm = 1 µSv/h
 */
static uint32_t nanoSievertPerH(uint32_t cpm)
{
	return ((1000*cpm ) /(29*6) );
}



static USB_DEV_INFO DeviceInfo;
static HID_DEVICE_INFO HidDevInfo;
static ROM **  rom = (ROM **)0x1fff1ff8;

typedef struct usbhid_out_s
{
	uint32_t perMin;
	uint32_t totalCount;
	uint32_t totalSec;
	uint32_t nanoSievertPerH;
}usbhid_out_t;



void usbHIDSetOutReport (uint8_t dst[], uint32_t length)
{
  // DO NOTHING, no data from PC
}

void usbHIDGetInReport (uint8_t src[], uint32_t length);


static void intro(int num){
#if 0
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

	delayms_queue(23*7);

  } while (--num);
#endif
}

static void getGeigerMeshVal()
{
	MPKT * mpkt= meshGetMessage('g');
	//char buf[32];
	if (MO_TIME(mpkt->pkt)!=0) {
		lcdPrint("Mesh:");
		lcdPrintln((char*)MO_BODY(mpkt->pkt));
		//lcdPrintln("cpm");
	}
}

/* USB String Descriptor (optional) */
#define WBVAL(x) ((x) & 0xFF),(((x) >> 8) & 0xFF)
const uint8_t USB_HIDStringDescriptor1[] =
{
  /* Index 0x00: LANGID Codes */
  0x04,                              /* bLength */
  USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
  WBVAL(0x0409), /* US English */    /* wLANGID */
  /* Index 0x04: Manufacturer */
  0x1C,                              /* bLength */
  USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
  'C',0,
  'C',0,
  'C',0,
  ' ',0,
  ' ',0,
  ' ',0,
  ' ',0,
  ' ',0,
  ' ',0,
  ' ',0,
  ' ',0,
  ' ',0,
  ' ',0,
  /* Index 0x20: Product */
  0x28,                              /* bLength */
  USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
  'r',0,
  '0',0,
  'k',0,
  'e',0,
  't',0,
  ' ',0,
  'g',0,
  'e',0,
  'i',0,
  'g',0,
  'e',0,
  'r',0,
  ' ',0,
  ' ',0,
  ' ',0,
  ' ',0,
  ' ',0,
  ' ',0,
  ' ',0,
  /* Index 0x48: Serial Number */
  0x1A,                              /* bLength */
  USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
  '0',0,
  '0',0,
  '0',0,
  '0',0,
  '0',0,
  '0',0,
  '0',0,
  '0',0,
  '0',0,
  '0',0,
  '0',0,
  '0',0,
  /* Index 0x62: Interface 0, Alternate Setting 0 */
  0x0E,                              /* bLength */
  USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
  'H',0,
  'I',0,
  'D',0,
  ' ',0,
  ' ',0,
  ' ',0,
};

void usbHIDInit (void)
{
  // Setup USB clock
  SCB_PDRUNCFG &= ~(SCB_PDSLEEPCFG_USBPAD_PD);        // Power-up USB PHY
  SCB_PDRUNCFG &= ~(SCB_PDSLEEPCFG_USBPLL_PD);        // Power-up USB PLL

  SCB_USBPLLCLKSEL = SCB_USBPLLCLKSEL_SOURCE_MAINOSC; // Select PLL Input
  SCB_USBPLLCLKUEN = SCB_USBPLLCLKUEN_UPDATE;         // Update Clock Source
  SCB_USBPLLCLKUEN = SCB_USBPLLCLKUEN_DISABLE;        // Toggle Update Register
  SCB_USBPLLCLKUEN = SCB_USBPLLCLKUEN_UPDATE;

  // Wait until the USB clock is updated
  while (!(SCB_USBPLLCLKUEN & SCB_USBPLLCLKUEN_UPDATE));

  // Set USB clock to 48MHz (12MHz x 4)
  SCB_USBPLLCTRL = (SCB_USBPLLCTRL_MULT_4);
  while (!(SCB_USBPLLSTAT & SCB_USBPLLSTAT_LOCK));    // Wait Until PLL Locked
  SCB_USBCLKSEL = SCB_USBCLKSEL_SOURCE_USBPLLOUT;

  // Set USB pin functions
  //IOCON_PIO0_1 &= ~IOCON_PIO0_1_FUNC_MASK;
  //IOCON_PIO0_1 |= IOCON_PIO0_1_FUNC_CLKOUT;           // CLK OUT
  IOCON_PIO0_3 &= ~IOCON_PIO0_3_FUNC_MASK;
  IOCON_PIO0_3 |= IOCON_PIO0_3_FUNC_USB_VBUS;         // VBus
  IOCON_PIO0_6 &= ~IOCON_PIO0_6_FUNC_MASK;
  IOCON_PIO0_6 |= IOCON_PIO0_6_FUNC_USB_CONNECT;      // Soft Connect

  // Disable internal resistor on VBUS (0.3)
  //gpioSetPullup(&IOCON_PIO0_3, gpioPullupMode_Inactive); not available in L0dable
  IOCON_PIO0_3= (IOCON_PIO0_3 & ~IOCON_COMMON_MODE_MASK)|gpioPullupMode_Inactive;


  // HID Device Info
  volatile int n;
  HidDevInfo.idVendor = USB_VENDOR_ID;
  HidDevInfo.idProduct = USB_PROD_ID;
  HidDevInfo.bcdDevice = USB_DEVICE;
  HidDevInfo.StrDescPtr = (uint32_t)USB_HIDStringDescriptor1[0];
  HidDevInfo.InReportCount = sizeof(usbhid_out_t);
  HidDevInfo.OutReportCount = 0;
  HidDevInfo.SampleInterval = 0x20;
  HidDevInfo.InReport = usbHIDGetInReport;
  HidDevInfo.OutReport = usbHIDSetOutReport;

  DeviceInfo.DevType = USB_DEVICE_CLASS_HUMAN_INTERFACE;
  DeviceInfo.DevDetailPtr = (uint32_t)&HidDevInfo;

  /* Enable Timer32_1, IOCON, and USB blocks (for USB ROM driver) */
  SCB_SYSAHBCLKCTRL |= (SCB_SYSAHBCLKCTRL_CT32B1 | SCB_SYSAHBCLKCTRL_IOCON | SCB_SYSAHBCLKCTRL_USB_REG);

  /* Use pll and pin init function in rom */
  /* Warning: This will also set the system clock to 48MHz! */
   //(*rom)->pUSBD->init_clk_pins();

  /* insert a delay between clk init and usb init */
  for (n = 0; n < 75; n++) {__asm("nop");}

  (*rom)->pUSBD->init(&DeviceInfo); /* USB Initialization */
  (*rom)->pUSBD->connect(true);     /* USB Connect */
}

static inline void usbHidDisconnect(void) {
	(*rom)->pUSBD->connect(false); /* USB Disconnect */
}


UINT perMin;
uint32_t startTime;
static void transmitGeigerMeshVal(uint32_t cpm,uint32_t time)
{
	if (perMin>0) {
	MPKT * mpkt= meshGetMessage('g');
	MO_TIME_set(mpkt->pkt,time);
	strcpy((char*)MO_BODY(mpkt->pkt),IntToStr(cpm,5,0));
	strcpy((char*)(MO_BODY(mpkt->pkt)+strlen((char*)MO_BODY(mpkt->pkt)))," cpm");
	lcdPrint("S:");
	lcdPrintln((char*)MO_BODY(mpkt->pkt));
	}
}


static uint8_t mainloop() {
	uint32_t volatile oldCount = IntCtr;
	perMin = 0; // counts in last 60 s
	uint32_t minuteTime = _timectr;
	startTime = minuteTime;
	uint8_t button;
	usbHIDInit();
	while (1) {
		LED_OFF;
		lcdClear();
		lcdPrintln("   Geiger");
		lcdPrintln("   Counter");
		// ####
		for (int i = 0; i < (14 * (_timectr - minuteTime)) / (60 * 100); i++) {
			lcdPrint("#");
		}
		lcdPrintln("");
		lcdPrint(" ");
		lcdPrintInt(IntCtr);
		lcdPrint(" in ");
		lcdPrintInt((_timectr - startTime) / 100);
		lcdPrintln("s");
		lcdPrint(" ");
		lcdPrintInt(perMin);
		lcdPrintln(" cpm");
		{
			uint32_t equivalent = nanoSievertPerH(perMin);
			lcdPrint(" ");
			lcdPrintInt(equivalent / 1000);
			lcdPrint(".");
			lcdPrintInt((equivalent % 1000) / 100);
			lcdPrintInt((equivalent % 100) / 10);
			lcdPrintInt((equivalent % 10));
			lcdPrintln(" uSv/h");

		}
		if (GetVoltage() < MIN_SAFE_VOLTAGE) {
			if (GetVoltage() < CRIT_VOLTAGE) {
				lcdPrintln("Battery CRIT!");
			} else
				lcdPrintln("Battery low");
		} else lcdPrintln(" ");
		// remember: We have a 10ms Timer counter
		if ((minuteTime + 60 * 100) < _timectr) {
			// dumb algo: Just use last 60 seconds count
			perMin = IntCtr - oldCount;
			minuteTime = _timectr;
			oldCount = IntCtr;
			transmitGeigerMeshVal(perMin,minuteTime / (100));
		}
		getGeigerMeshVal();
		lcdRefresh();
		delayms_queue_plus(42, 0);
		button = getInputRaw();
		if (button != BTN_NONE) {
			break;
		}
	}
	usbHidDisconnect();
	return button;
}


void usbHIDGetInReport (uint8_t src[], uint32_t length)
{
  usbhid_out_t out;

  out.perMin=perMin;
  out.totalSec=(_timectr-startTime)/100;
  out.totalCount=IntCtr;
  out.nanoSievertPerH=nanoSievertPerH(perMin);

  memcpy(src,&out,sizeof(out));
}


