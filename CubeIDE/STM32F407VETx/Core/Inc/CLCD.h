#ifndef _TEXTLCD_H
#define _TEXTLCD_H

#include "stm32f4xx_hal.h"
#include "main.h"
#include <string.h>

/*/////   PIN CONNECTION   /////*/
/*		CLCD_RS		PE0			*/
/*		CLCD_RW 	PE1			*/
/*		CLCD_EN 	PE2			*/
/*		CLCD_D4 	PE4			*/
/*		CLCD_D5 	PE5			*/
/*		CLCD_D6 	PE6			*/
/*		CLCD_D7 	PE7			*/
////////////////////////////////*/

#define GPIO_RS GPIOE
#define GPIO_RW GPIOE
#define GPIO_EN GPIOE
#define GPIO_D4	GPIOE
#define GPIO_D5	GPIOE
#define GPIO_D6	GPIOE
#define GPIO_D7	GPIOE

#define GPIO_PIN_RS	GPIO_PIN_0
#define GPIO_PIN_RW	GPIO_PIN_1
#define GPIO_PIN_EN	GPIO_PIN_2
#define GPIO_PIN_D4	GPIO_PIN_4
#define GPIO_PIN_D5	GPIO_PIN_5
#define GPIO_PIN_D6	GPIO_PIN_6
#define GPIO_PIN_D7	GPIO_PIN_7

enum {
	CLCD_ROW0 = 0,
	CLCD_ROW1 = 1,
	CLCD_MAX_ROW = 2
};

enum {
	CLCD_COL0 = 0,
	CLCD_COL1 = 1,
	CLCD_COL2 = 2,
	CLCD_COL3 = 3,
	CLCD_COL4 = 4,
	CLCD_COL5 = 5,
	CLCD_COL6 = 6,
	CLCD_COL7 = 7,
	CLCD_COL8 = 8,
	CLCD_COL9 = 9,
	CLCD_COL10 = 10,
	CLCD_COL11 = 11,
	CLCD_COL12 = 12,
	CLCD_COL13 = 13,
	CLCD_COL14 = 14,
	CLCD_COL15 = 15,
	CLCD_MAX_COL = 16
};

void CLCD_GPIO_Init(void);
void CLCD_Write_Instruction(unsigned char b);
void CLCD_Write_Display(unsigned char b);
void CLCD_Gotoxy(unsigned char x, unsigned char y);
void CLCD_Puts(unsigned char x, unsigned char y, unsigned char *str);
void CLCD_Init(void);
void CLCD_Clear(void);

#define	MAX_CLCD_BUF_LEN	(CLCD_MAX_ROW*CLCD_MAX_COL)

#define	CLCD_Printf(fmt,...)	do{\
	unsigned char	x = CLCD_COL0;\
	unsigned char	y = CLCD_ROW0;\
	sprintf((char*)CLCD_buf,fmt,__VA_ARGS__);\
	CLCD_Clear();\
	for(int i=0;i<min(strlen((char*)CLCD_buf),MAX_CLCD_BUF_LEN);i++){\
		if(CLCD_buf[i]=='\n'){\
			x=0,y++;\
			continue;\
		}\
		CLCD_Gotoxy(x,y);CLCD_Write_Display(CLCD_buf[i]);\
		if(++x==CLCD_MAX_COL)x=0,y++;\
	}\
}while(0);

extern uint8_t	CLCD_buf[];
#endif
