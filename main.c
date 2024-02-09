#include <avr/io.h>
#include <avr/sfr_defs.h>
#include <string.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <util/delay.h>
# define F_CPU 1000000
int dooropen=0;
int doorclose=0;
int moving=0;

#define LCD_Dir DDRA					/* Define LCD data port direction */
#define LCD_Port PORTA					/* Define LCD data port */
#define RS PA1							/* Define Register Select (data reg./command reg.) signal pin */
#define EN PA2 							/* Define Enable signal pin */

void LCD_Command( unsigned char cmnd )
{
	LCD_Port = (LCD_Port & 0b10000111) | ((cmnd & 0xF0)>>1); /* sending upper nibble */
	LCD_Port &= ~ (1<<RS);				/* RS=0, command reg. */
	LCD_Port |= (1<<EN);				/* Enable pulse */
	_delay_us(1);
	LCD_Port &= ~ (1<<EN);

	_delay_us(200);

	LCD_Port = (LCD_Port &  0b10000111) | (cmnd << 3);  /* sending lower nibble */
	LCD_Port |= (1<<EN);
	_delay_us(1);
	LCD_Port &= ~ (1<<EN);
	_delay_ms(2);
}



void LCD_Init (void)					/* LCD Initialize function */
{
	LCD_Dir = 0xFF;						/* Make LCD command port direction as o/p */
	_delay_ms(200);						/* LCD Power ON delay always >15ms */
	
	LCD_Command(0x33);
	LCD_Command(0x32);		    		/* send for 4 bit initialization of LCD  */
	LCD_Command(0x28);              	/* Use 2 line and initialize 5*7 matrix in (4-bit mode)*/
	LCD_Command(0x0c);              	/* Display on cursor off*/
	LCD_Command(0x06);              	/* Increment cursor (shift cursor to right)*/
	LCD_Command(0x01);              	/* Clear display screen*/
	_delay_ms(2);
	LCD_Command (0x80);					/* Cursor 1st row 0th position */
}


void LCD_String (char *str)				/* Send string to LCD function */
{
	int i;
	for(i=0;str[i]!=0;i++)				/* Send each char of string till the NULL */
	{
		LCD_Char (str[i]);
	}
}

void LCD_Char( unsigned char data )
{
	LCD_Port = (LCD_Port &  0b10000111) | ((data & 0xF0)>>1); /* sending upper nibble */
	LCD_Port |= (1<<RS);				/* RS=1, data reg. */
	LCD_Port|= (1<<EN);
	_delay_us(1);
	LCD_Port &= ~ (1<<EN);

	_delay_us(200);

	LCD_Port = (LCD_Port &  0b10000111) | (data << 3); /* sending lower nibble */
	LCD_Port |= (1<<EN);
	_delay_us(1);
	LCD_Port &= ~ (1<<EN);
	_delay_ms(2);
}
void LCD_Clear()
{
	LCD_Command (0x01);					/* Clear display */
	_delay_ms(2);
	LCD_Command (0x80);					/* Cursor 1st row 0th position */
}

///flame sensor interruptvector
ISR(INT0_vect) {
	LCD_Clear();
	LCD_String("welcoemrgency");
	stationary();
	
}




/////

void setup()
{
	DDRD|=(1<<PD3);// trig pin as an output
	DDRD&=~(1<<PD4);//echo is an input pin
}



float pulsecalculation()
{

	int i=0;
	
	TCCR0|=(1<<CS00)|(1<<WGM01);//no prescalar with CTC mode
	OCR0=60;
	TCNT0=0;
	
	//PORTD|=(1<<PD2);
	
	PORTD|=(1<<PD3);
	_delay_us(3000);
	PORTD&=~(1<<PD3);
	while(bit_is_set(PIND,4))
	{
		if(bit_is_set(TIFR,OCF0))
		{
			i++;
		}
	}
	return i;
}
///doors opening and closing functions
void opendoor()
{
	
	PORTC|=(1<<PC4);
	_delay_ms(4000);//// time needs experimental determination ASAP
	PORTC&=~(1<<PC4);
	_delay_ms(4000);
	
	
}

void closedoor()
{
	
	
	PORTC|=(1<<PC5);
	_delay_ms(4000);//// time needs experimental determination ASAP
	PORTC&=~(1<<PC5);
	_delay_ms(4000);
	
	
}


//dynamic motion functions
void moveup()   /////pre defined function to move the elevator up via a timed manner
{
	PORTC&=~(1<<PC6);
	PORTC&=~(1<<PC7);
	
	PORTC|=(1<<PC7);
	_delay_ms(3000);
	PORTC&=~(1<<PC6);
	PORTC&=~(1<<PC7);
	_delay_ms(3000);
}


void movedown()  /////pre defined function to move the elevator down via a timed manner
{
	PORTC&=~(1<<PC6);
	PORTC&=~(1<<PC7);
	
	PORTC|=(1<<PC6);
	_delay_ms(2000);
	PORTC&=~(1<<PC6);
	PORTC&=~(1<<PC7);
	_delay_ms(2000);
	
}


void stationary() /////pre defined function to stop the elevator
{
	PORTC&=~(1<<PC7);
	PORTC&=~(1<<PC6);
	///display("stationary");
	_delay_ms(3000);
}

void sensorpart()
{
	float t;
	t=pulsecalculation();
	
	for(int z=0;z<2;z++)
	{
		if(t>900 &&t<1200)
		{
			
			PORTC|=(1<<PC2);
			_delay_ms(5000);
			
		}
		
		
		
		else if(t<1100)
		{
			PORTC=0x00;
		}
		
	}
}


void emergency()
{
	PORTC|=(1<<PC5);
	LCD_String("welcoemergency");

}

void shutdownemergency()
{
	PORTA&=~(1<<PC5);
	LCD_String("welcoWorking");

}



int main(void)
{
	LCD_Init();
	LCD_Clear();							/* Initialization of LCD*/
	LCD_String("welcoWorking");
	setup();////remove setup
	DDRC=0xff;
	PORTC=0x00;
	
	DDRB=0x00;
	PORTB=0xff;
	
	DDRD&=~(1<<PD2);//make the pd2 input for the flame sensor
	PORTD&=~(1<<PD2);
	// Trigger INT0 on any logical change
	MCUCR |= (1 << ISC00);
	
	// Enable INT0 interrupt
	GICR |= (1 << INT0);
	
	int indicator;
	int poistion=0;
	//position vectors
	int startinglocation;
	int finalocation;
	float directionmovementvector;
	///timer work
	
	int flag;
	sei();
	/* Replace with your application code */
	while (1)
	{
		
		PORTC&=~(1<<PC5);
		
		directionmovementvector=0;
		poistion=poistion;
		finalocation=finalocation;
		indicator=indicator;
		/// for loop to determine the movement vector
		
		
		
		for(int i=0;i<3;i++)
		{
			PORTC=0xff;
			_delay_ms(3000);
			PORTC=0x00;
			_delay_ms(5000);
			if(bit_is_clear(PINB,i))///button at PC0 has been pressed
			{
				startinglocation=poistion;
				finalocation=i;
				directionmovementvector=finalocation-startinglocation;
				indicator=finalocation;
			}
		}
		
		if(directionmovementvector==0)///the elevator is intially at the first floor
		{
			
			for(int yu=0;yu<550;yu++)
			{
				sensorpart();
			}
			dooropen=1;
			stationary();
			poistion=poistion;
		}
		
		else if( directionmovementvector==1)///the elevator is expexted to move up by one floor
		{
			
			for(int yw=0;yw<550;yw++)
			{
				sensorpart();
			}
			closedoor();
			dooropen=0;
			moveup();
			dooropen=1;
			poistion=indicator;
			opendoor();
		}
		
		
		else if(directionmovementvector==2)
		{
			for(int yui=0;yui<550;yui++)
			{
				sensorpart();
			}
			
			closedoor();
			dooropen=0;
			moveup();
			_delay_ms(800);
			moveup();
			
			poistion=2;
			opendoor();
			dooropen=1;
			stationary();
			poistion=poistion;
		}
		
		else if(directionmovementvector==-1)
		{
			for(int yuip=0;yuip<550;yuip++)
			{
				sensorpart();
			}
			//sensorpart();
			closedoor();
			dooropen=0;
			movedown();
			poistion=indicator;
			opendoor();
			dooropen=1;
		}
		else if(directionmovementvector==-2)
		{
			for(int yuipp=0;yuipp<550;yuipp++)
			{
				sensorpart();
			}
			closedoor();
			dooropen=0;
			movedown();
			_delay_ms(500);
			movedown();
			opendoor();
			dooropen=1;
			poistion=indicator;
		}

		
		
		PORTC&=~(1<<PC5);

	}
}