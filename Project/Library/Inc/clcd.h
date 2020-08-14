#ifndef CLCD_H_
#define CLCD_H_

#include "board.h"
#include <stdio.h>
#include <stdarg.h>

//DDRAM 설정 : 0x80+DDRAM address 값

#define LINE1	0x80  //첫번째 라인의 DDRAM address : 0x00 -> 0x80+0x00
#define LINE2	0xC0  //두번째 라인의 DDRAM address : 0x40 -> 0x80+0x40
#define LINE3	0x94  //세번째 라인의 DDRAM address : 
#define LINE4	0xD4  //네번째 라인의 DDRAM address : 


//PG2=RS, PG1=LCD_EN, PG0=RW, 에 연결
/*
#define LCD_CON     PORTG //RS, EN, RW 는 PORTG에 연결
#define LCD_DATA    PORTC //DATA 포트는 PORTC에 연결
#define LCD_DATA_DIR DDRC //DATA 포트 입출력 설정
#define LCD_DATA_IN PINC  //DATA 포트 입력 방향 설정
*/

#define RS_1		PORTD |= 0x01 	//RS -> PD0에 연결, set
#define RS_0		PORTD &= ~0x01	//clear
#define RW_1		PORTD |= 0x02		//RW -> PD1에 연결, set
#define RW_0		PORTD &= ~0x02	//clear
#define E_1			PORTD |= 0x04		//Enable -> PD2에 연결. set
#define E_0			PORTD &= ~0x04	//clear

//LCD 명령어들
//#define DIS_R	0x1C	//0001 11_ _ display(S/C=1) 를 오른쪽(R/L=1)으로 쉬프트.
//#define DIS_L	0x18	//0001 10_ _ display(S/C=1) 를 왼쪽(R/L=0)으로 쉬프트.
#define RIGHT	0x1C	//0001 01_ _ cursor(S/C=0) 를 오른쪽(R/L=1)으로 쉬프트.
#define LEFT	0x18	//0001 00_ _ cursor(S/C=0) 를 왼쪽(R/L=0)으로 쉬프트.
#define CLEAR	0x02	//화면 지우기
#define FUNCTION 0x28
#define ENTRY	0x06
#define DISPLAY	0x0C

char LCD_BusyCheck(U8 temp);
void LCD_cmd_write(char cmd);
U8 LCD_cmd_read(char cmd);
void LCD_data_write(char *data);
void LCD_wr_string(char d_line, char *lcd_str);
void LCD_printf(char d_line, char * msg,...);
void CLCD_init(void);

#endif
