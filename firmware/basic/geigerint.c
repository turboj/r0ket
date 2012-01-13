#include <sysinit.h>
#include <string.h>
#include <time.h>

#include "basic/basic.h"
#include "basic/byteorder.h"
#include "basic/config.h"

#include "lcd/lcd.h"
#include "lcd/print.h"


uint32_t volatile BusIntCtr __attribute__ ((aligned))=0;




void businterrupt(void) {
		BusIntCtr++;
}
