/*
 * clcd.h
 *
 *  Created on: 2020. 8. 13.
 *      Author: Kiwon
 */

#ifndef INC_CLCD_H_
#define INC_CLCD_H_

#include	"gpio.h"

#define LINE1	0x80
#define LINE2	0xC0

#define RS_1		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_SET)
#define RS_0		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_RESET)
#define RW_1		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_1,GPIO_PIN_SET)
#define RW_0		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_1,GPIO_PIN_RESET)
#define E_1			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2,GPIO_PIN_SET)
#define E_0			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2,GPIO_PIN_RESET)

#define RIGHT	0x1C	//0001 01_ _ cursor(S/C=0) 를 오른쪽(R/L=1)으로 쉬프트.
#define LEFT	0x18	//0001 00_ _ cursor(S/C=0) 를 왼쪽(R/L=0)으로 쉬프트.
#define CLEAR	0x02	//화면 지우기
#define FUNCTION 0x28
#define ENTRY	0x06
#define DISPLAY	0x0C

extern void LCD_cmd_write(char cmd);
extern void LCD_data_write(char *data);
extern void LCD_wr_string(char d_line, char *lcd_str);
extern void CLCD_init(void);

#endif /* INC_CLCD_H_ */
