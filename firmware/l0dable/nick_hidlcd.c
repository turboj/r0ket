/**
 * @file hidlcd.c
 *
 * @date 05.01.2012
 * 
 * @author turboj
 *
 * Display control via HID
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
#define USB_PROD_ID 0x2012
#define USB_DEVICE 0x101

void usbHIDInit (void);
static uint8_t mainloop();


void ram(void) {

	mainloop();

}



static USB_DEV_INFO DeviceInfo;
static HID_DEVICE_INFO HidDevInfo;
static ROM **  rom = (ROM **)0x1fff1ff8;



uint32_t volatile delayTime;
uint8_t volatile do_refresh;

typedef struct usbhid_out_s {
	uint16_t offset;
	uint8_t count;
	uint8_t flags;
	uint8_t data[58];
} usbhid_out_t;


void usbHIDSetOutReport (uint8_t dst[], uint32_t length)
{
  usbhid_out_t * out=(usbhid_out_t *)dst;
  uint8_t flags=out->flags;
  if (out->count) {
	  memcpy(&lcdBuffer[out->offset],out->data, out->count);
  }
  if (flags & 1) {
	  delayTime=out->offset;
  }

  if (flags&0x80) {
	  do_refresh=1;
  }

}



void usbHIDGetInReport (uint8_t src[], uint32_t length) {
	// no data to PC
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
  'l',0,
  'c',0,
  'd',0,
  ' ',0,
  'h',0,
  'i',0,
  'd',0,
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
  HidDevInfo.StrDescPtr = (uint32_t)&USB_HIDStringDescriptor1[0];
  HidDevInfo.InReportCount = sizeof(usbhid_out_t);
  HidDevInfo.OutReportCount = sizeof(usbhid_out_t);
  HidDevInfo.SampleInterval = 0x1;
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

static uint8_t mainloop() {
	uint8_t button;
	usbHIDInit();
	delayTime=42;
	lcdClear();
	lcdPrintln("");
	lcdPrintln("HID LCD");
	do_refresh=1;
	while (1) {
		if (do_refresh) {
			lcdRefresh();
			do_refresh=0;
		}
		delayms_queue_plus(delayTime, 0);
		button = getInputRaw();
		if (button != BTN_NONE) {
			break;
		}
	}
	usbHidDisconnect();
	return button;
}
