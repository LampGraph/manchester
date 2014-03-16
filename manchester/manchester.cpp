/*   Sample program for Olimex AVR-P40 with ATMega16 processor
 *   Echoes back the received characters on the uart. In order to work,
 *   connect the RX pad with PD1(pin 15) and TX pad with PD0(pin 14)
 *   Compile with AVRStudio+WinAVR (gcc version 3.4.6)
 */

#define   __AVR_ATmega328__   1
#define OSCSPEED   16000000      /* in Hz */

#include "avr/io.h"
#include "avr/delay.h"
#include "avr/interrupt.h"                                                                                                                             

void Initialize(void); //Установка начальных значений
int Send_data(unsigned char ch); //Отправка данных
int Send_byte(unsigned char ch);
int Send_pause(int i);
void Timer_init();
void Timer_stop();

volatile bool gen_flag=0;
volatile uint8_t timer_count;

void Initialize(void)
{
   PORTB = 0x1;
   PORTC = 0x0;
   PORTD = 0x0;

   DDRB = 0x1;
   DDRC = 0x0;
   DDRD = 0x0;
}

void USART_Init(uint32_t baud )
	{
	UBRR0 = 0;
	/* Setting the XCKn port pin as output, enables master mode. */
	DDRD |= (0<<1);
	/* Set MSPI mode of operation and SPI data mode 0. */
	UCSR0C = (1<<UMSEL01)|(1<<UDORD0)|(1<<UCPHA0);
	/* Enable receiver and transmitter. */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);	
	/* Set baud rate. */	
	/* IMPORTANT: The Baud Rate must be set after the transmitter is enabled
	*/	
	unsigned int BaudRate = OSCSPEED / (16 * baud) - 1;
	UBRR0H = (unsigned char) (BaudRate>>8);
	UBRR0L = (unsigned char) BaudRate;

//   UCSR0B = UCSR0B | 0b00011000;
//   UCSR0C = UCSR0C | 0b10000110;
}

unsigned char UARTReceive(void)
{
	while ( !(UCSR0A & (1<<RXC0)) );
	/* Get and return received data from buffer */
	return UDR0;
}

void UARTTransmit(unsigned char Data)
{
   while (!(UCSR0A & 0b00100000));
   UDR0 = Data;
}

int main()
{
   unsigned char ch;
//   Initialize();
   USART_Init(9600);
//   Timer_init();
   while (1)
   {  
//      ch = UARTReceive();      
	_delay_ms(100);
	ch=0x3D;
	  if (ch)
      {
         UARTTransmit(ch);
		
		 Send_data(ch);

      }
   }   
}

int Send_data(unsigned char ch)
{
	unsigned char a=0xff;	
	uint8_t i=0;
	bool pulse;	
	Timer_init();
		
	//пауза 89ms
	Send_pause(89);	//Посылаем паузу 89ms
	Send_byte(a); //стартовый байт
	Send_byte(ch); //Данные
	Send_pause(89);
	
	Timer_stop();
//	UARTTransmit(a);
}

void Timer_init() 
{
	TCCR0A |= (1<<COM0A0);// Режим счетчика обычный, изменение значения вывода OC0A (PortD.6) на противоположное
	TCCR0B |= (1<<CS00); //Задаем несущую частоту 31,25кГц
	DDRD|=(1<<6);// Установить вывод OC0A (PortD.6) на выход
	SREG |=(1<<7); // Глобальное разрешение прерываний.
	TIMSK0 |= (1<<TOIE0); //прерывание по переполнению
}

void Timer_stop()
{
	TCCR0B &= ~(1<<CS00);
}

ISR(TIMER0_OVF_vect) //Обработчик прерывания по переполнению счетчика
{
//	gen_flag != gen_flag;
	timer_count++;
	if (timer_count>=64)	//Добиваемся длительности сигнала 1,780ms/2=889ns
	{
		timer_count=0;
		gen_flag = gen_flag?false:true;	
/*			if (gen_flag)
			{
				gen_flag=false;
				PORTB |= (1<<5);
			} else
			{
				gen_flag=true;
				PORTB &= ~(1<<5);
			}
*/
	}

	TCNT0=61; //Установкой начального значения счетчика добиваемся максимальной стабильной несущей частоты 36Кгц
}

int Send_byte (unsigned char ch)
{
	//отправляем байт данных
	for (int i=0 ; i<8 ; i++)
	{
		while (!gen_flag);
		if (ch & 0x80)
		{
			PORTB |= (1<<5);
			while (gen_flag);
			PORTB &= ~(1<<5);
		} else
		{
			PORTB &= ~(1<<5);
			while (gen_flag);
			PORTB |= (1<<5);
		}
		ch <<= 1;
	}
}

int Send_pause(int i)
{	
	int k=i/0.889; //Считаем количество тиков
	i=0;
	PORTB &= ~(1<<5);
	while (i<=k) {
		if (!gen_flag)
		{
			while (!gen_flag);
		} else
		{
			while (gen_flag);
		}
		i++;
	}
	PORTB |= (1<<5);
}


void temp()
{
	int i;
}