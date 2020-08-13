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

  //PORTD=(cmd & 0xF0);		//ó�� 8bit ������ �� �� 4�� ���� ����
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

  //PORTD=((cmd<<4) & 0xF0);		// 4�� ������ shift�ϰ� ������ 4�� ����
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
	LCD_cmd_write(d_line); //���ڿ��� ǥ���ϱ� ���� ���� ����
	while(*lcd_str != '\0')
	{
		LCD_data_write(lcd_str);//�Ѱ��� ���ھ� LCD�� ǥ���Ѵ�.
		lcd_str++;
	}
}

void CLCD_init(void)
{
  HAL_Delay(20);        //15msec �̻� �ð�����
  LCD_cmd_write(0x30);	//��ɼ�(�����͹��� 8��Ʈ, ���μ�:2��), FUNCTION

  HAL_Delay(5);         //4.1msec �̻� �ð�����, ��������
  LCD_cmd_write(0x30);	//��ɼ�, ���� ����, FUNCTION
  HAL_Delay(1);       //100usec �̻� �ð�����, ��������
  LCD_cmd_write(0x30);	//��ɼ�, ���� ����, FUNCTION
  LCD_cmd_write(FUNCTION);  //
  LCD_cmd_write(DISPLAY);  //ǥ�� On

  HAL_Delay(1);	//wait for 40us
  LCD_cmd_write(CLEAR);  //ȭ�� �����
  HAL_Delay(2); //wait for 1.53 ms
  LCD_cmd_write(ENTRY);  //��Ʈ������
}
