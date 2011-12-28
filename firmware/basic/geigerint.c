#include <sysinit.h>
#include <string.h>
#include <time.h>

#include "basic/basic.h"
#include "basic/byteorder.h"
#include "basic/config.h"

#include "lcd/lcd.h"
#include "lcd/print.h"

#define LED_ON	GPIO_GPIO1DATA |= (1 << 7)
#define LED_OFF GPIO_GPIO1DATA &= ~(1 << 7)

uint32_t volatile BusIntCtr __attribute__ ((aligned))=0;




void businterrupt(void) {



//		IOCON_PIO1_11 = 0;
		LED_ON;
//		GPIO_GPIO1DATA |= (1 << 11);
		BusIntCtr++;
}
