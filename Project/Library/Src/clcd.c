#include "clcd.h"
#include "board.h"


//LCD Busy flag check 함수
char LCD_BusyCheck(U8 temp)
{
	if(temp & 0x80) return 1;
	else 			return 0;
}

//LCD에 명령을 쓰기 위한 함수
void LCD_cmd_write(char cmd)
{
  /*PORTG = CMD_WRITE;  //PORTG에 RS, E, RW가 연결되어 있다.
  PORTC = cmd; //PORTB에 데이터버스가 연결되어 있다.
  PORTG = PORTG^LCD_EN;//E 신호를 H->L로 하기 위해
  _delay_ms(2); //LCD 내부 동작시간*/
  _delay_ms(1);

  PORTD=(cmd & 0xF0);		//처음 8bit 데이터 중 앞 4개 먼저 보냄
  RS_0;
  RW_0;
 
  _delay_us(1);
  E_1;
  _delay_us(1);
  E_0;

  PORTD=((cmd<<4) & 0xF0);		// 4개 앞으로 shift하고 나머지 4개 보냄
  RS_0;
  RW_0;
 
  _delay_us(1);
  E_1;
  _delay_us(1);
  E_0;
}

//LCD에 데이터를 쓰기 위한 함수
void LCD_data_write(char *data)
{
  /*PORTG = DATA_WRITE;  //PORTE에 RS, E, RW가 연결되어 있다.
  PORTC = *data; //PORTC에 데이터버스가 연결되어 있다.
  PORTG = PORTG^LCD_EN;//E 신호를 H->L로 하기 위해
  _delay_ms(2); //LCD 내부 동작시간*/
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

// LCD에 문자열을 표시하기 위한 함수
void LCD_wr_string(char d_line, char *lcd_str)
{
	LCD_cmd_write(d_line); //문자열을 표시하기 위한 라인 설정
	while(*lcd_str != '\0')
	{
		LCD_data_write(lcd_str);//한개의 문자씩 LCD에 표시한다.
		lcd_str++;
	}
}
/* CLCD로 printf 함수 사용하기. */
void LCD_printf(char d_line, char * msg,...)
{
  volatile	U8 LCDStr[30];	//unsigned char 8-bit.
  
  LCD_cmd_write(d_line); //문자열을 표시하기 위한 라인 설정
  
  /* vsprintf 쓰기 위한 필수. va_list va_start va_end. */
  va_list ap;
  va_start(ap, msg);
  vsprintf((void*)&LCDStr[0], msg,ap);
  va_end(ap);

  /* vsprintf는 LCDStr[i]를 하나씩 보내주고
   밑에 있는 LCD_String()은 LCDStr[]을 받아서 LCD로 뿌려준다.*/
  LCD_wr_string(d_line,(void *)&LCDStr[0]);
}

// ATmega128의 포트 초기화
// LCD 초기화, 초기화 강좌의 순서도 참조
void CLCD_init(void)
{
	cli();

  _delay_ms(10);        //15msec 이상 시간지연
  	LCD_cmd_write(0x30);	//기능셋(데이터버스 8비트, 라인수:2줄), FUNCTION

  _delay_ms(2);         //4.1msec 이상 시간지연, 생략가능
  	LCD_cmd_write(0x30);	//기능셋, 생략 가능, FUNCTION
  _delay_us(100);       //100usec 이상 시간지연, 생략가능
  	LCD_cmd_write(0x30);	//기능셋, 생략 가능, FUNCTION
  	LCD_cmd_write(FUNCTION);  //
 	LCD_cmd_write(DISPLAY);  //표시 On

 _delay_us(40);	//wait for 40us
  LCD_cmd_write(CLEAR);  //화면 지우기
 _delay_us(1530); //wait for 1.53 ms
  LCD_cmd_write(ENTRY);  //엔트리모드셋
  

  sei();
}
