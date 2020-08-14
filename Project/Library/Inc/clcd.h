#ifndef CLCD_H_
#define CLCD_H_

#include "board.h"
#include <stdio.h>
#include <stdarg.h>

//DDRAM ���� : 0x80+DDRAM address ��

#define LINE1	0x80  //ù��° ������ DDRAM address : 0x00 -> 0x80+0x00
#define LINE2	0xC0  //�ι�° ������ DDRAM address : 0x40 -> 0x80+0x40
#define LINE3	0x94  //����° ������ DDRAM address : 
#define LINE4	0xD4  //�׹�° ������ DDRAM address : 


//PG2=RS, PG1=LCD_EN, PG0=RW, �� ����
/*
#define LCD_CON     PORTG //RS, EN, RW �� PORTG�� ����
#define LCD_DATA    PORTC //DATA ��Ʈ�� PORTC�� ����
#define LCD_DATA_DIR DDRC //DATA ��Ʈ ����� ����
#define LCD_DATA_IN PINC  //DATA ��Ʈ �Է� ���� ����
*/

#define RS_1		PORTD |= 0x01 	//RS -> PD0�� ����, set
#define RS_0		PORTD &= ~0x01	//clear
#define RW_1		PORTD |= 0x02		//RW -> PD1�� ����, set
#define RW_0		PORTD &= ~0x02	//clear
#define E_1			PORTD |= 0x04		//Enable -> PD2�� ����. set
#define E_0			PORTD &= ~0x04	//clear

//LCD ��ɾ��
//#define DIS_R	0x1C	//0001 11_ _ display(S/C=1) �� ������(R/L=1)���� ����Ʈ.
//#define DIS_L	0x18	//0001 10_ _ display(S/C=1) �� ����(R/L=0)���� ����Ʈ.
#define RIGHT	0x1C	//0001 01_ _ cursor(S/C=0) �� ������(R/L=1)���� ����Ʈ.
#define LEFT	0x18	//0001 00_ _ cursor(S/C=0) �� ����(R/L=0)���� ����Ʈ.
#define CLEAR	0x02	//ȭ�� �����
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
