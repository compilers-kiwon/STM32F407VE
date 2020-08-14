#include "clcd.h"
#include "board.h"


//LCD Busy flag check �Լ�
char LCD_BusyCheck(U8 temp)
{
	if(temp & 0x80) return 1;
	else 			return 0;
}

//LCD�� ����� ���� ���� �Լ�
void LCD_cmd_write(char cmd)
{
  /*PORTG = CMD_WRITE;  //PORTG�� RS, E, RW�� ����Ǿ� �ִ�.
  PORTC = cmd; //PORTB�� �����͹����� ����Ǿ� �ִ�.
  PORTG = PORTG^LCD_EN;//E ��ȣ�� H->L�� �ϱ� ����
  _delay_ms(2); //LCD ���� ���۽ð�*/
  _delay_ms(1);

  PORTD=(cmd & 0xF0);		//ó�� 8bit ������ �� �� 4�� ���� ����
  RS_0;
  RW_0;
 
  _delay_us(1);
  E_1;
  _delay_us(1);
  E_0;

  PORTD=((cmd<<4) & 0xF0);		// 4�� ������ shift�ϰ� ������ 4�� ����
  RS_0;
  RW_0;
 
  _delay_us(1);
  E_1;
  _delay_us(1);
  E_0;
}

//LCD�� �����͸� ���� ���� �Լ�
void LCD_data_write(char *data)
{
  /*PORTG = DATA_WRITE;  //PORTE�� RS, E, RW�� ����Ǿ� �ִ�.
  PORTC = *data; //PORTC�� �����͹����� ����Ǿ� �ִ�.
  PORTG = PORTG^LCD_EN;//E ��ȣ�� H->L�� �ϱ� ����
  _delay_ms(2); //LCD ���� ���۽ð�*/
	_delay_ms(1);

	PORTD=(*data & 0xF0);
	RS_1;
	RW_0;
	_delay_us(1);
	E_1;
	_delay_us(1);
	E_0;

	PORTD=((*data<<4) & 0xF0);
	RS_1;
	RW_0;
	_delay_us(1);
	E_1;
	_delay_us(1);
	E_0;
}

// LCD�� ���ڿ��� ǥ���ϱ� ���� �Լ�
void LCD_wr_string(char d_line, char *lcd_str)
{
	LCD_cmd_write(d_line); //���ڿ��� ǥ���ϱ� ���� ���� ����
	while(*lcd_str != '\0')
	{
		LCD_data_write(lcd_str);//�Ѱ��� ���ھ� LCD�� ǥ���Ѵ�.
		lcd_str++;
	}
}
/* CLCD�� printf �Լ� ����ϱ�. */
void LCD_printf(char d_line, char * msg,...)
{
  volatile	U8 LCDStr[30];	//unsigned char 8-bit.
  
  LCD_cmd_write(d_line); //���ڿ��� ǥ���ϱ� ���� ���� ����
  
  /* vsprintf ���� ���� �ʼ�. va_list va_start va_end. */
  va_list ap;
  va_start(ap, msg);
  vsprintf((void*)&LCDStr[0], msg,ap);
  va_end(ap);

  /* vsprintf�� LCDStr[i]�� �ϳ��� �����ְ�
   �ؿ� �ִ� LCD_String()�� LCDStr[]�� �޾Ƽ� LCD�� �ѷ��ش�.*/
  LCD_wr_string(d_line,(void *)&LCDStr[0]);
}

// ATmega128�� ��Ʈ �ʱ�ȭ
// LCD �ʱ�ȭ, �ʱ�ȭ ������ ������ ����
void CLCD_init(void)
{
	cli();

  _delay_ms(10);        //15msec �̻� �ð�����
  	LCD_cmd_write(0x30);	//��ɼ�(�����͹��� 8��Ʈ, ���μ�:2��), FUNCTION

  _delay_ms(2);         //4.1msec �̻� �ð�����, ��������
  	LCD_cmd_write(0x30);	//��ɼ�, ���� ����, FUNCTION
  _delay_us(100);       //100usec �̻� �ð�����, ��������
  	LCD_cmd_write(0x30);	//��ɼ�, ���� ����, FUNCTION
  	LCD_cmd_write(FUNCTION);  //
 	LCD_cmd_write(DISPLAY);  //ǥ�� On

 _delay_us(40);	//wait for 40us
  LCD_cmd_write(CLEAR);  //ȭ�� �����
 _delay_us(1530); //wait for 1.53 ms
  LCD_cmd_write(ENTRY);  //��Ʈ������
  

  sei();
}
