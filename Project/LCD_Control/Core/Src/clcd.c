/*
 * clcd.c
 *
 *  Created on: 2020. 8. 13.
 *      Author: Kiwon
 */

#include	"clcd.h"

void LCD_cmd_write(char cmd)
{
  HAL_Delay(1);

  //PORTD=(cmd & 0xF0);		//처음 8bit 데이터 중 앞 4개 먼저 보냄
  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,(cmd&0x80)>>7);
  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,(cmd&0x40)>>6);
  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,(cmd&0x20)>>5);
  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,(cmd&0x10)>>4);
  RS_0;
  RW_0;

  HAL_Delay(1);
  E_1;
  HAL_Delay(1);
  E_0;

  //PORTD=((cmd<<4) & 0xF0);		// 4개 앞으로 shift하고 나머지 4개 보냄
  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,(cmd&0x08)>>3);
  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,(cmd&0x04)>>2);
  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,(cmd&0x02)>>1);
  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,(cmd&0x01)>>0);
  RS_0;
  RW_0;

  HAL_Delay(1);
  E_1;
  HAL_Delay(1);
  E_0;
}

void LCD_data_write(char *data)
{
	HAL_Delay(1);

	//PORTD=(*data & 0xF0);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,(*data&0x80)>>7);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,(*data&0x40)>>6);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,(*data&0x20)>>5);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,(*data&0x10)>>4);
	RS_1;
	RW_0;
	HAL_Delay(1);
	E_1;
	HAL_Delay(1);
	E_0;

	//PORTD=((*data<<4) & 0xF0);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,(*data&0x08)>>3);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,(*data&0x04)>>2);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,(*data&0x02)>>1);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,(*data&0x01)>>0);
	RS_1;
	RW_0;
	HAL_Delay(1);
	E_1;
	HAL_Delay(1);
	E_0;
}

void LCD_wr_string(char d_line, char *lcd_str)
{
	LCD_cmd_write(d_line); //문자열을 표시하기 위한 라인 설정
	while(*lcd_str != '\0')
	{
		LCD_data_write(lcd_str);//한개의 문자씩 LCD에 표시한다.
		lcd_str++;
	}
}

void CLCD_init(void)
{
  HAL_Delay(20);        //15msec 이상 시간지연
  LCD_cmd_write(0x30);	//기능셋(데이터버스 8비트, 라인수:2줄), FUNCTION

  HAL_Delay(5);         //4.1msec 이상 시간지연, 생략가능
  LCD_cmd_write(0x30);	//기능셋, 생략 가능, FUNCTION
  HAL_Delay(1);       //100usec 이상 시간지연, 생략가능
  LCD_cmd_write(0x30);	//기능셋, 생략 가능, FUNCTION
  LCD_cmd_write(FUNCTION);  //
  LCD_cmd_write(DISPLAY);  //표시 On

  HAL_Delay(1);	//wait for 40us
  LCD_cmd_write(CLEAR);  //화면 지우기
  HAL_Delay(2); //wait for 1.53 ms
  LCD_cmd_write(ENTRY);  //엔트리모드셋
}
