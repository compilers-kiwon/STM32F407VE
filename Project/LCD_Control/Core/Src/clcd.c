/*
 * clcd.c
 *
 *  Created on: 2020. 8. 13.
 *      Author: Kiwon
 */

#include	"clcd.h"

void CLCD_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIOE Periph clock enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();

	/* Configure RS, RW, EN, D4, D5, D6, D7 in output pushpull mode */
	GPIO_InitStruct.Pin = GPIO_PIN_RS;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIO_RS, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_RW;
	HAL_GPIO_Init(GPIO_RW, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_EN;
	HAL_GPIO_Init(GPIO_EN, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_D4;
	HAL_GPIO_Init(GPIO_D4, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_D5;
	HAL_GPIO_Init(GPIO_D5, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_D6;
	HAL_GPIO_Init(GPIO_D6, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_D7;
	HAL_GPIO_Init(GPIO_D7, &GPIO_InitStruct);
}

void CLCD_Write_Instruction(unsigned char b)
{
	// send MSB 4 bits to CLDC
	HAL_GPIO_WritePin(GPIO_D7,GPIO_PIN_D7,(b&0x80)>>7);
	HAL_GPIO_WritePin(GPIO_D6,GPIO_PIN_D6,(b&0x40)>>6);
	HAL_GPIO_WritePin(GPIO_D5,GPIO_PIN_D5,(b&0x20)>>5);
	HAL_GPIO_WritePin(GPIO_D4,GPIO_PIN_D4,(b&0x10)>>4);

	// notice an instruction to CLCD
	HAL_GPIO_WritePin(GPIO_RS,GPIO_PIN_RS,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIO_RW,GPIO_PIN_RW,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIO_EN,GPIO_PIN_EN,GPIO_PIN_RESET);

	HAL_GPIO_WritePin(GPIO_EN,GPIO_PIN_EN,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIO_EN,GPIO_PIN_EN,GPIO_PIN_RESET);

	// send LSB 4 bits to CLCD
	HAL_GPIO_WritePin(GPIO_D7,GPIO_PIN_D7,(b&0x08)>>3);
	HAL_GPIO_WritePin(GPIO_D6,GPIO_PIN_D6,(b&0x04)>>2);
	HAL_GPIO_WritePin(GPIO_D5,GPIO_PIN_D5,(b&0x02)>>1);
	HAL_GPIO_WritePin(GPIO_D4,GPIO_PIN_D4,(b&0x01)>>0);

	// notice an instruction to CLCD
	HAL_GPIO_WritePin(GPIO_RS,GPIO_PIN_RS,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIO_RW,GPIO_PIN_RW,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIO_EN,GPIO_PIN_EN,GPIO_PIN_RESET);

	HAL_GPIO_WritePin(GPIO_EN,GPIO_PIN_EN,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIO_EN,GPIO_PIN_EN,GPIO_PIN_RESET);

	HAL_Delay(1);
}

void CLCD_Write_Display(unsigned char b)
{
	// send MSB 4 bits to CLDC
	HAL_GPIO_WritePin(GPIO_D7,GPIO_PIN_D7,(b&0x80)>>7);
	HAL_GPIO_WritePin(GPIO_D6,GPIO_PIN_D6,(b&0x40)>>6);
	HAL_GPIO_WritePin(GPIO_D5,GPIO_PIN_D5,(b&0x20)>>5);
	HAL_GPIO_WritePin(GPIO_D4,GPIO_PIN_D4,(b&0x10)>>4);

	// notice the data to CLCD
	HAL_GPIO_WritePin(GPIO_RS,GPIO_PIN_RS,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIO_RW,GPIO_PIN_RW,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIO_EN,GPIO_PIN_EN,GPIO_PIN_RESET);

	HAL_GPIO_WritePin(GPIO_EN,GPIO_PIN_EN,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIO_EN,GPIO_PIN_EN,GPIO_PIN_RESET);

	// send LSB 4 bits to CLDC
	HAL_GPIO_WritePin(GPIO_D7,GPIO_PIN_D7,(b&0x08)>>3);
	HAL_GPIO_WritePin(GPIO_D6,GPIO_PIN_D6,(b&0x04)>>2);
	HAL_GPIO_WritePin(GPIO_D5,GPIO_PIN_D5,(b&0x02)>>1);
	HAL_GPIO_WritePin(GPIO_D4,GPIO_PIN_D4,(b&0x01)>>0);

	// notice the data to CLCD
	HAL_GPIO_WritePin(GPIO_RS,GPIO_PIN_RS,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIO_RW,GPIO_PIN_RW,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIO_EN,GPIO_PIN_EN,GPIO_PIN_RESET);

	HAL_GPIO_WritePin(GPIO_EN,GPIO_PIN_EN,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIO_EN,GPIO_PIN_EN,GPIO_PIN_RESET);

	HAL_Delay(1);
}


void CLCD_Gotoxy(unsigned char col, unsigned char row)
{
	// the first line indicator : 0x80
	// the second line indicator : 0xC0
	// column is just offset from a line indicator
	switch(row)
	{
		case 0 :
			CLCD_Write_Instruction(0x80+col);
			break;
		case 1 :
			CLCD_Write_Instruction(0xC0+col);
			break;
		default:
			// do nothing
			break;
	}
}

void CLCD_Puts(unsigned char col, unsigned char row, unsigned char *str)
{
	CLCD_Gotoxy(col,row);

	for(uint32_t i=0;str[i]!='\0';i++)
	{
		CLCD_Write_Display(str[i]);
	}
}

void CLCD_Init(void)
{
	HAL_Delay(100);
	CLCD_Write_Instruction(0x28);
	HAL_Delay(10);
	CLCD_Write_Instruction(0x28);
	HAL_Delay(10);
	CLCD_Write_Instruction(0x0C);
	CLCD_Write_Instruction(0x06);
	CLCD_Write_Instruction(0x02);
	CLCD_Write_Instruction(0x01);
	CLCD_Write_Instruction(0x01);
}

void CLCD_Clear(void)
{
	CLCD_Write_Instruction(0x01);
	HAL_Delay(10);
}
