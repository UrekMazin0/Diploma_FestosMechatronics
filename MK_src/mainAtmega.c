/*
 * Diod_test.c
 *
 * Created: 01.12.2020 23:23:49
 * Author : feanor
 */ 
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// HZ MK
#define F_CPU 8000000UL
// UART SETTINGS
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1

#define HeaderInput 0x01
#define HeaderOutput 0xFF

void packet0x01(void);
void packet0x02(void);
void packet0x03(void);
void packet0x04(void)c;
void packet0x05(void); 

#define bufSize 255
unsigned char inputData [bufSize];
unsigned char inputDataCount = 0;
unsigned char outputData [bufSize];
unsigned char outputDataCount = 0;

unsigned int lenPacket = 0;
unsigned char CRC = 0;
unsigned char count0x10 = 0; //debug value

void USART_Init( unsigned int ubrr );
void USART_Transmit( unsigned char data );
void U0_Puts (char S[]);

void Init(void);

void deletePacket(unsigned int endPos);
void deleteHeader(unsigned char Pos);
unsigned char ControlSumm ( unsigned char *Mass, unsigned int length);

ISR (USART_RX_vect)
{
	data=UDR0;
	//USART_Transmit(data);
	inputData[inputDataCount] = data;
	inputDataCount++;
		
	if ((inputData[0]==HeaderInput) && (inputDataCount>3)) //Если первый символ является заголовком
	{
		lenPacket = inputData[1];
		if (inputDataCount == lenPacket) //Проверяем длину пакета 
		{
			//Если длина пакета верная
            CRC = ControlSumm(&inputData[0], lenPacket);
			if((CRC == inputData[lenPacket]) && (lenPacket <= bufSize)) //Проверяем целостность пакета
			{
                count0x10++;
                //Если пакет верный производим поиск комманды
                switch (inputData[3])
                {
                    case 0x01: packet0x01();
                    break;
                    case 0x02: packet0x02();
                    break;
                    case 0x03: packet0x03();
                    break;
                    case 0x04: packet0x04();
                    break;
                    case 0x05: packet0x05(&inputData[Pos+3]);
                    break;
                }
                deletePacket(lenPacket+1); //Удаляем пакет
            } 
            else
            {
                //Если контрольная сумма не соглась то удаляем неверный заголовок
                deleteHeader(0);
            }
		}
		else
		{
			//Если длина пакета неверная то удаляем заголовок
			deleteHeader(0);
		}
	} 
	else
	{
		//Если первый символ не является заголовком - удаляем его
		while ((inputData[0]!=Header)&&(inputDataCount>0)) 
		{
			deleteHeader(0);
		}
	}
}

int main(void)
{
	Init();
	
    while (1) 
    {
		//_delay_ms(250);
		//PORTC = PORTC || 0b00000001;
		//_delay_ms(250);
		//PORTC = PORTD && 0b00000000;
    }
}

void Init(void)
{
	// Port D initialization
	// Function: Bin0=Out
	DDRC = (1<<DDC0);
	// State: Bit0=T
	PORTC =(0<<PORTC0);
	
	// Global enable interrupts
	asm("sei");
	
	USART_Init ( MYUBRR ); // usart inizialization
}

void USART_Init( unsigned int ubrr )
{
	/* Set baud rate */
	
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	/* Set frame format: 8data, 2stop bit */
	UCSR0C = (1<<USBS0)|(3<<UCSZ00);
	/* Enable receiver and transmitter */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);
	
}

void USART_Transmit( unsigned char data )
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) );
	/* Put data into buffer, sends the data */
	UDR0 = data;
}

void U0_Puts (char S[])
{
	while ( !( UCSR0A & (1<<UDRE0)) );
	unsigned char length = strlen(S);
	for (unsigned char i=0;i<length;i++) USART_Transmit(S[i]);
	//USART_Transmit('\r');
	//USART_Transmit('\n');
}

/////////////////////////////////////////////////////////////////////////////
void deleteHeader(unsigned char Pos)
{
	for (unsigned int i=Pos;i<bufSize-1;i++) inputData[i]=inputData[i+1];
	inputData[bufSize]=0;
	inputDataCount --;
}

void deletePacket(unsigned int endPos)
{
	for (unsigned int i=endPos;i<bufSize;i++)
	{
		inputData[i-endPos]=inputData[i];
		inputData[i]=0;
	}
	inputDataCount = inputDataCount - endPos;
}

unsigned char ControlSumm (unsigned char *Mass, unsigned int length)
{
	unsigned int output;
	output = *(Mass);
	for (unsigned int i=1;i<length;i++) output = output + *(Mass+i);
	return output;
}

int getHeaderPos(void)
{
	int output =-1;
	for (unsigned int i=0;i<inputDataCount;i++) 
		if (inputData[i]==Header)
		{
			output = i;
			break;
		}
	return output;
}

void packet0x01(void) //Отправка информации об устройстве
{
    char packetLength = 0x05;
	outputData[0] = HeaderOutput;
	outputData[1] = packetLength;
	outputData[2] = 0x00;
	outputData[3] = 0x01; // deviceType
    outputData[4] = 0x00; // trash for databuffer
	
	outputData[5] = ControlSumm(&outputData,5);
	
	for (unsigned char i=0;i<6;i++) USART_Transmit(outputData[i]);	
}
