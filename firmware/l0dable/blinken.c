// written by @monkeydom at cccamp2011

#include <sysinit.h>

#include "basic/basic.h"
#include "lcd/lcd.h"
#include "lcd/display.h"
#include "lcd/print.h"
#include "filesystem/ff.h"
#include "filesystem/select.h"

#include "usetable.h"

#define SAVE_KEY_REPEAT delayms(100)

void showHDL(void);
uint8_t showHDLBLM(char *filename, bool loop);
void showAllMovies();
void mainScreen();

//shows *.lcd
void ram(void)
{
    char filename[13];
    char key;

    while (1) {
    	SAVE_KEY_REPEAT;
    	mainScreen();
		key = getInputWait();
	
		// Show file
		if (key == BTN_RIGHT) {
			// Select file
			lcdClear();
			delayms(230);
			selectFile(filename,"BLM");
			// Load as animation
			showHDLBLM(filename, true);
		} else if (key == BTN_DOWN) {    
			// Select file
			lcdClear();
			//delayms(230);
			showAllMovies();
		} else if (key == BTN_ENTER) {
			showHDL();
		} else if (key == BTN_LEFT) {
			// Exit
			return;
		}
	}
}

void mainScreen() {
	getInputWaitRelease();
	while (!getInputRaw()) {
		lcdLoadImage("blinken0.lcd"); // 800ms
		lcdRefresh();
		getInputWaitTimeout(800);
		if (getInputRaw()) return;
		lcdLoadImage("blinken1.lcd"); // 200ms
		lcdRefresh();
		getInputWaitTimeout(200);
	}
}

void drawWindow(int x, int y, int on) {
	int originX = x * 5+4;
	int originY = y * 8+3;
	for (int x = originX; x < originX+ 4; x++) {
		for (int y = originY; y < originY+5; y++) {
			lcdSetPixel(x,y,on);
		}
	}
}

void showHDL(void) {
//    char blinkenHDLFrame[8][18] = {
//    	{0,0,0,0,1,1,1,0,0,0,0,1,1,1,0,0,0,0},
//		{0,0,1,1,1,1,1,1,0,0,1,1,1,1,1,1,0,0},
//		{0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0},
//		{0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0},
//		{0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0},
//		{0,0,0,0,1,1,1,1,1,1,1,1,1,1,0,0,0,0},
//		{0,0,0,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0},
//		{0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0}
//    };
    char key;
    
	lcdFill(0);
	while (1) {
		for (int x=0; x<18; x++) {
			for (int y=0; y<8; y++) {
				drawWindow(x,y,true);//blinkenHDLFrame[y][x]);
			}
		}
		lcdDisplay();
		
		SAVE_KEY_REPEAT;
		key = getInputWait();
		if (key == BTN_LEFT) return;
		for (int x=0; x<18; x++) {
			for (int y=0; y<8; y++) {
				drawWindow(x,y,false);//1-blinkenHDLFrame[y][x]);
			}
		}
		lcdDisplay();
		
		SAVE_KEY_REPEAT;
		key = getInputWait();
		if (key == BTN_LEFT) return;

	}
}

#define SEARCH_AT 1
#define PARSE_DURATION 2
#define GOTO_NEWLINE 3
#define PARSE_FRAME 4

#define MAX_BUF_SIZE (18+20)*2

void showHDLFrame(bool frame[8][18]) {
//	lcdFill(0);
	for (int x=0; x<18; x++) {
		for (int y=0; y<8; y++) {
			drawWindow(x,y,frame[y][x]);
		}
	}
	lcdDisplay();
}

#define PERPAGE 10
#define FLEN 13

void showAllMovies() {
	while(1) {
		int skip = 0;
        char files[PERPAGE][FLEN];
        int count = getFiles(files, PERPAGE, skip, "BLM");
		for (int filePos = 0; filePos < count; filePos++) {
			showHDLBLM(files[filePos],false);
			if (getInputRaw() == BTN_LEFT) return;
		}
		skip++;
		if (count == 0) {
			skip = 0;
		}
	}
}

uint8_t showHDLBLM(char *filename, bool loop) {
	FIL file;
	int result;
	UINT readBytes;
	char buffer[MAX_BUF_SIZE];
	UINT buffer_offset = 0;
	uint8_t state = SEARCH_AT;
	int duration = 0;
	bool frame[8][18];
	char key=0;
	int8_t framex = 0;
	int8_t framey = 0;
	
	
	int frameCount = 0;
	int frameCountFile = 0;
	lcdClear();
	lcdFill(0);

	result = f_open(&file, filename, FA_OPEN_EXISTING|FA_READ);
	if (result) return 1;
	
	while (1) {
		result = f_read(&file,buffer,MAX_BUF_SIZE,&readBytes);
//		lcdPrintInt(readBytes);
//		lcdPrintln("");
		if (readBytes == 0) {
			if (loop) {
				f_lseek(&file,0);
				frameCountFile = 0;
				continue;
			} else {
				return 0;
			}
		}
		if (result) return -1;
//		lcdPrintln("Read state...");
		buffer_offset = 0;
		while (buffer_offset < readBytes) {
			switch (state) {
				case SEARCH_AT:
//					lcdPrint(IntToStr(frameCountFile,3,0));
//					lcdPrint(IntToStr(frameCount,2,0));
//					lcdPrint("sa");
					while (buffer_offset < readBytes &&
					       buffer[buffer_offset] != '#' &&
					       buffer[buffer_offset] != '@') buffer_offset++;
					if (buffer[buffer_offset] == '#') {
						state = GOTO_NEWLINE;
					} else if (buffer[buffer_offset] == '@') {
						gpioSetValue (RB_LED1, 1); 
						state = PARSE_DURATION;
						duration = 0;
					}
					buffer_offset++;
					break;
				case GOTO_NEWLINE:
//					lcdPrint(IntToStr(frameCountFile,3,0));
//					lcdPrint(IntToStr(frameCount,2,0));
//					lcdPrint("gn");
					while (buffer_offset < readBytes &&
					       buffer[buffer_offset] != '\n') buffer_offset++;
					if (buffer_offset < readBytes && buffer[buffer_offset] == '\n') {
						state = SEARCH_AT;
						buffer_offset++;
					}
					break;
				case PARSE_DURATION:
////					lcdPrint(IntToStr(frameCountFile,3,0));
////					lcdPrint(IntToStr(frameCount,2,0));
////					lcdPrint("pd");
					while (buffer_offset < readBytes &&
					       buffer[buffer_offset] >= '0' && buffer[buffer_offset] <='9') {
						duration = duration * 10 + (buffer[buffer_offset++] - '0');
					}

					if (buffer_offset != readBytes) {
						state = PARSE_FRAME;
						framex = 0;
						framey = -1;
					}
					break;
				case PARSE_FRAME:
//					lcdPrint(IntToStr(frameCountFile,3,0));
//					lcdPrint("-");
//					lcdPrint(IntToStr(frameCount,2,0));
//					lcdPrint("pf");
					while (buffer_offset < readBytes) {
						char byte = buffer[buffer_offset];
						if (byte == '\n') {
							framey++;
							framex=0;
							if (framey >= 9) {
								frameCount++;
								frameCountFile++;
								// draw it baby
								showHDLFrame(frame);
								key = getInputWaitTimeout(duration > 50 ? duration : 50);
								gpioSetValue (RB_LED1, 0); 
								if (key == BTN_LEFT) return 0;
								else getInputWaitRelease();
								state = SEARCH_AT;
								break;
							}
						} else if (byte == '0' || byte == '1') {
							frame[framey][framex++] = (byte != '0');
						}
						buffer_offset++;
					}
					break;
			}
		}
	}
	
}




