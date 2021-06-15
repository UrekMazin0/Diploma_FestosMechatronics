/*
 * sdka.c
 *
 * Created: 19.01.2016 14:35:04
 * Author : Homer_000
 */ 


#include "Config.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdio.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>
#include "libTWI.h"
#include "libDS1337.h"
#include "libPWMSearcher.h"
#include "libEeprom.h"

#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1
#define ADC_VREF_TYPE ((1<<REFS1) | (1<<REFS0) | (0<<ADLAR))


void (*funcptr)( void ) = 0x0000;//Программный RESET 

#define PowerOn PORTC |= (1<<PORTC1) // Включение внешнего питания 
#define PowerOff PORTC  &= ~(1<<PORTC1) //Выключение внешнего питания

//#define  BPowerOn PORTD |= (1<<PORTD3) // Включение питания Bluetooth
//#define  BPowerOff PORTD  &= ~(1<<PORTD3) //Выключение питания Bluetooth

#define Down 0
#define Up 1
#define Header 0x02 //Заголовок пакета
#define EepromSize 1024 //Размер eeprom в байтах

//device info
#define MaxDeviceType 0x02 //Максимальный код устройства
#define MaxFirmwareVersion 0x0A //Максимальная версия прошивки
#define MaxHardwareVersion 0x01 //Максимальная версия железа
uint8_t deviceType;
uint8_t EEMEM  deviceType_e;
uint8_t firmwareVersion;
uint8_t EEMEM  firmwareVersion_e;
uint8_t haedwareVersion;
uint8_t EEMEM  haedwareVersion_e;
uint8_t serialNumber[5];
uint8_t EEMEM serialNumber_e[5];
#define typeSealer 0x01 //Код герметизатора
#define typeSearch 0x02 //Код поискового устройства

//battery status
#define currentMinLeveBat1 1800
#define currentMaxLeveBat1 3300
#define currentMinLeveBat2 6000
#define currentMaxLeveBat2 24000
unsigned int bat1;//Батарея часов
unsigned int bat2;//Внешняя батарея
uint16_t MinBat1;
uint16_t EEMEM MinBat1_e;
uint16_t MaxBat1;
uint16_t EEMEM MaxBat1_e;
uint16_t MinBat2;
uint16_t EEMEM MinBat2_e;
uint16_t MaxBat2;
uint16_t EEMEM MaxBat2_e;

//Точки
#define PointsCount 10 //Количество точек
#define pointPrefixAddres 0x0f
#define sizeOfStructPoint  0x0A
uint8_t hours[PointsCount];
uint8_t EEMEM hours_e[PointsCount];
uint8_t minutes[PointsCount];
uint8_t EEMEM minutes_e[PointsCount];
uint8_t seconds[PointsCount];
uint8_t EEMEM seconds_e[PointsCount];
uint8_t days[PointsCount];
uint8_t EEMEM days_e[PointsCount];
uint8_t month[PointsCount];
uint8_t EEMEM month_e[PointsCount];
uint8_t year[PointsCount];
uint8_t EEMEM year_e[PointsCount];
uint8_t config[PointsCount];
uint8_t EEMEM config_e[PointsCount];
uint8_t power[PointsCount];
uint8_t EEMEM power_e[PointsCount];
uint16_t VBat[PointsCount];
uint16_t EEMEM VBat_e[PointsCount];

//Основной алгоритм рбаоты
#define MaxTransmitMode 0x02
uint8_t currentPointNumber; //Текущая точка
uint8_t EEMEM currentPointNumber_e; //Текущая точка
uint8_t isRun; //Состояние внешнего питания
uint8_t EEMEM isRun_e; //Состояние внешнего питания
uint8_t currentPower; //Текущая мощность передатчика
uint8_t EEMEM currentPower_e;//Текущая мощность передатчика
uint8_t currentPWMMin;
uint8_t EEMEM currentPWMMin_e;
uint8_t currentPWMMax;
uint8_t EEMEM currentPWMMax_e;
uint8_t currentExtPower; //Состояние внешнего питания
uint8_t EEMEM currentExtPower_e; //Состояние внешнего питания
uint8_t currentBPower; //Состояние питание bluetooth модуля
uint8_t EEMEM currentBPower_e; //Состояние питание bluetooth модуля
uint8_t countSignal; //Количество переудов в пачке импульсов
uint8_t EEMEM countSignal_e; //Количество переудов в пачке импульсов
uint8_t countSignalPause; //Количество переудов в паузе
uint8_t EEMEM countSignalPause_e; //Количество переудов в паузе
uint8_t currentTransmitMode; //Режим работы передатчика
uint8_t EEMEM currentTransmitMode_e; //Режим работы передатчика
uint32_t currentMSequence; //Текущая М-последовательность
uint32_t EEMEM currentMSequence_e; //Текущая М-последовательность
uint8_t currentMSequenceLen; //Длина текущей М-последовательности
uint8_t EEMEM currentMSequenceLen_e; //Длина текущей М-последовательности
uint16_t getNextPoint(void); //Найти следующую точку



void Init(void);
unsigned int read_adc(unsigned char adc_input);
unsigned int checkBat (void);
unsigned int checkResBat (void);
void USART_Init( unsigned int ubrr );
void USART_Transmit( unsigned char data );
void U0_Puts (char S[]);
void InitTWI(void);
void InitDS1337 (void);
void goSleep (unsigned char mode);
unsigned char ControlSumm (unsigned char *Mass, unsigned int length);
int getHeaderPos(void);
void deletePacket(unsigned int endPos);
void clearInputData(void);
void deleteHeader(unsigned char Pos);
void sendPowerState(void); //Отправить состояние питания
void setPowerExt(uint8_t Power); //Установить состояние внешнего питания
void handingMissingPoints(uint8_t *startPoint,uint8_t *endPoint); //Обработка пропущенных точек
void setBpower(uint8_t Power); //Установить питание bluetooth модуля
void executeCurrentPoint(void); //Выполнить текущую точку
void packet0x01(void);
void packet0x02(void);
void packet0x03(void);
void packet0x04(void);
void packet0x05(unsigned char *input);
void packet0x06(unsigned char *input);
void packet0x08(unsigned char *input);
void packet0x09(void);
void packet0x0A(void);
void packet0x0B(unsigned char *input);
void packet0x0C(unsigned char *input);
void packet0x0D(void);
void packet0x0E(unsigned char *input);
void packet0x0F(unsigned char *input);
void packet0x10(void);
void packet0x11(void);
void packet0x12(void); //Прочитать минимальный уровень первой батареи
void packet0x13(unsigned char *input); //Установить минимальный уровень первой батареи
void packet0x14(void); //Прочитать минимальный уровень второй батареи
void packet0x15(unsigned char *input); //Установить минимальный уровень второй батареи
void packet0x16(unsigned char *input); //Записать информацию об устройстве
void packet0x17(void); //Получить текущее состояние
void packet0x18(unsigned char *input); //Запустить алгоритм
void packet0x19(void); //Остановить алгортм
void packet0x1A(void); //Прочитать текущую мощность передатчика
void packet0x1B(unsigned char *input); //Установить текущую мощность передатчика
void packet0x1C(void); //Прочитать параметры ШИМ
void packet0x1D(unsigned char *input); //Установить параметры ШИМ
void packet0x1E(void); //Прочитать регистры RTC
void packet0x1F(unsigned char *input); //Записать регистры RTC
void packet0x20(void); //Чтение состояния прерываний
void packet0x21(void); //Очистка eeprom
void packet0x22(void); //Прочитать режим работы передатчика
void packet0x23(unsigned char *input); //Установить режим работы передатчика
void packet0x24(void); //Прочитать параметры передатчика
void packet0x25(unsigned char *input); //Установить параметры передатчика
void packet0x26(void); //Прочитать параметры М-последовательности
void packet0x27(unsigned char *input); //Установить параметры М-последовательности

void writeStructDeviceInfo(void);
void readStructDeviceInfo(void);
void readPoints(void);
void writePoints(void);
unsigned char SLP EEMEM;
uint16_t VBatMin;
uint16_t EEMEM VBatMin_e;


unsigned int A;
char Str[20];
char data;
#define bufSize 255
unsigned char inputData [bufSize];
unsigned char inputDataCount=0; 
unsigned char outputData [bufSize];
unsigned char outputDataCount=0;
int Pos;
unsigned int lenPacket=0;
unsigned char CRC=0;
unsigned char BB;
unsigned char TWIStatus;

Time_ time;
Date_ date;

//debug
unsigned char count0x10 = 0;

ISR (USART_RX_vect)
{
	data=UDR0;
	//USART_Transmit(data);
	inputData[inputDataCount] = data;
	inputDataCount++;
		
	if ((inputData[0]==Header)&&(inputDataCount>3)) //Если первый символ является заголовком
	{
		lenPacket = (inputData[2]<<8) + inputData[1];
		if (lenPacket<=bufSize) //Проверяем длину пакета 
		{
			//Если длиа акета верная
			if ((inputDataCount>=(Pos+lenPacket+1))&&(lenPacket<bufSize)) //Проверяем достаточно ли данных в буфере
			{
				CRC = ControlSumm(&inputData[0],Pos+lenPacket);
				if ((CRC == inputData[Pos+lenPacket])&&(inputDataCount>=lenPacket)) //Проверяем целостность пакета
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
						case 0x06: packet0x06(&inputData[Pos+3]);
						break;
						case 0x07: packet0x07(&inputData[Pos+3]);
						break;
						case 0x08: packet0x08(&inputData[Pos+3]);
						break;
						case 0x09: packet0x09();
						break;
						case 0x0A: packet0x0A();
						break;
						case 0x0B: packet0x0B(&inputData[Pos+3]);
						break;
						case 0x0C: packet0x0C(&inputData[Pos+3]);
						break;
						case 0x0D: packet0x0D();
						break;
						case 0x0E: packet0x0E(&inputData[Pos+3]);
						break;
						case 0x0F: packet0x0F(&inputData[Pos+3]);
						break;
						case 0x10: packet0x10();
						break;
						case 0x12: packet0x12();
						break;
						case 0x13: packet0x13(&inputData[Pos+3]);
						break;
						case 0x14: packet0x14();
						break;
						case 0x15: packet0x15(&inputData[Pos+3]);
						break;
						case 0x16: packet0x16(&inputData[Pos+3]);
						break;
						case 0x17: packet0x17();
						break;
						case 0x18: packet0x18(&inputData[Pos+3]);
						break;
						case 0x19: packet0x19();
						break;
						case 0x1A: packet0x1A();
						break;
						case 0x1B: packet0x1B(&inputData[Pos+3]);
						break;
						case 0x1C: packet0x1C();
						break;
						case 0x1D: packet0x1D(&inputData[Pos+3]);
						break;
						case 0x1E: packet0x1E();
						break;
						case 0x1F: packet0x1F(&inputData[Pos+3]);
						break;
						case 0x20: packet0x20();
						break;
						case 0x21: packet0x21();
						break;
						case 0x22: packet0x22();
						break;
						case 0x23: packet0x23(&inputData[Pos+3]);
						break;
						case 0x24: packet0x24();
						break;
						case 0x25: packet0x25(&inputData[Pos+3]);
						break;
						
						case 0x26: packet0x26();
						break;
						case 0x27: packet0x27(&inputData[Pos+3]);
						break;
						
					}
					deletePacket(lenPacket+1); //Удаляем пакет
				} else
					{
						//Если контрольная сумма не соглась то удаляем неверный заголовок
						deleteHeader(0);
					}
			
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

ISR (INT0_vect)
{	
	unsigned int ptr = 0xFF00;
	if (isRun)
	{
		config[currentPointNumber] = config[currentPointNumber] & 0x7F; //Сбрасываем флаг активности точки
		executeCurrentPoint();
	}
	
	
	uint8_t bufPointNumber = getNextPoint(); //Ищем следующую подходящую точку
	if ((bufPointNumber-currentPointNumber)>1) handingMissingPoints(&currentPointNumber,&bufPointNumber); 
	

	if ((bufPointNumber<PointsCount)&&(isRun)&&(bufPointNumber!=currentPointNumber)) //Если точка найдена
	{
		//Устанавливаем следующую точку
		currentPointNumber = bufPointNumber;
		Time_ currentTime;
		currentTime.Hours = hours[currentPointNumber];
		currentTime.Minutes = minutes[currentPointNumber];
		currentTime.Seconds = seconds[currentPointNumber];
		DS1337_SetAlarm1(&currentTime,days[currentPointNumber]); //Устанавливаем будильник
		packet0x09(); //Отправляем время будильника
		packet0x10(); //Отправляем все данные
	} else
	{
		isRun = false;	//Отключаем основной лагоритм
		eeprom_update_byte(&isRun_e,isRun);
		setBpower(0x01); //Включаем питание блютуз т.к. выполнена последняя точка
		packet0x10(); //Отправляем все данные
	}
	
	DS1337_ClearBit(0x0f,A1F); //Сбрасываем флаг будильника
	//DS1337_ClearBit(0x0E,A1IE); //Запрещаем прерывание
	packet0x11();
}

ISR (INT1_vect)
{
	U0_Puts("INT1\r\n");
}

ISR (PCINT0_vect)
{
	/*
	DS1337_GetTime(&time,DS1337_24H);
	if (time.TimeFormat == DS1337_AM)
	sprintf(Str,"\rTime %2d:%2d:%2d  AM",time.Hours,time.Minutes,time.Seconds);
	else if (time.TimeFormat == DS1337_PM)
	sprintf(Str,"\rTime %2d:%2d:%2d  PM",time.Hours,time.Minutes,time.Seconds);
	else sprintf(Str,"\rTime %2d:%2d:%2d    ",time.Hours,time.Minutes,time.Seconds);
	U0_Puts(Str);
		
	DS1337_GetDate(&date);
	sprintf(Str," Date %2d:%2d:%2d ",date.Day,date.Month,date.Year);
	U0_Puts(Str);
	
	sprintf(Str," Power %5d mV",checkBat());
	U0_Puts(Str);
	
		sprintf(Str," Bat %5d mV",checkResBat());
	U0_Puts(Str);
	*/
	
}

ISR (TWI_vect)
{
	BB = TWCR;
	U0_Puts("TWI");
	TWIStatus = TWSR;
	BB = TWCR;
	//TWCR = TWCR|(1<<TWINT);
 	TWCR = TWCR&(0<<TWSTA);	
	BB = TWCR;
}

int main(void)
{	
	unsigned char data;
	Init();
	
	readPoints();
	//BPowerOn;
	PORTD  &= ~(1<<PORTD4);
	eeprom_write_byte(&SLP,0); //Если первый запуск то обнуляем значение переменной
	readStructDeviceInfo();
    while (1) 
    {
		//sleep_cpu(); // спать!
		asm("SLEEP");
    }
}


void Init(void)
{	
	// Port B initialization
	// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
	DDRB=(0<<DDB7) | (0<<DDB6) | (0<<DDB5) | (0<<DDB4) | (0<<DDB3) | (0<<DDB2) | (0<<DDB1) | (0<<DDB0);
	// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=P Bit0=P
	PORTB=(0<<PORTB7) | (0<<PORTB6) | (0<<PORTB5) | (0<<PORTB4) | (0<<PORTB3) | (0<<PORTB2) | (1<<PORTB1) | (1<<PORTB0);
	
	// Port C initialization
	// Function: Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=Out Bit0=In
	DDRC=(0<<DDC6) | (0<<DDC5) | (0<<DDC4) | (1<<DDC3) | (1<<DDC2) | (1<<DDC1) | (0<<DDC0);
	// State: Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=0 Bit0=T
	PORTC=(0<<PORTC6) | (1<<PORTC5) | (1<<PORTC4) | (0<<PORTC3) | (0<<PORTC2) | (0<<PORTC1) | (0<<PORTC0);

	// Port D initialization
	// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=Out Bit2=In Bit1=In Bit0=In
	DDRD=(0<<DDD7) | (0<<DDD6) | (0<<DDD5) | (1<<DDD4) | (1<<DDD3) | (0<<DDD2) | (0<<DDD1) | (0<<DDD0);
	// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=0 Bit2=T Bit1=T Bit0=T
	PORTD=(0<<PORTD7) | (0<<PORTD6) | (0<<PORTD5) | (0<<PORTD4) | (0<<PORTD3) | (0<<PORTD2) | (0<<PORTD1) | (0<<PORTD0);
	
	
	// ADC initialization
	// ADC Clock frequency: 1000,000 kHz
	// ADC Voltage Reference: Int., cap. on AREF
	// ADC Auto Trigger Source: Free Running
	// Digital input buffers on ADC0: On, ADC1: On, ADC2: On, ADC3: On
	// ADC4: On, ADC5: On
	DIDR0=(0<<ADC5D) | (0<<ADC4D) | (0<<ADC3D) | (0<<ADC2D) | (0<<ADC1D) | (0<<ADC0D);
	ADMUX=ADC_VREF_TYPE;
	ADCSRA=(1<<ADEN) | (0<<ADSC) | (1<<ADATE) | (0<<ADIF) | (0<<ADIE) | (0<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
	ADCSRB=(0<<ADTS2) | (0<<ADTS1) | (0<<ADTS0);

	EIMSK=(0<<INT1) | (0<<INT0); //Выключаем внешнее прерывание
	// Global enable interrupts
	asm("sei");

	USART_Init ( MYUBRR );
	TWI_Init(50000);
	InitDS1337();	
	PWM_Init();
	
	// External Interrupt(s) initialization
	// INT0: On
	// INT0 Mode: Low level
	// INT1: Off
	// Interrupt on any change on pins PCINT0-7: On
	// Interrupt on any change on pins PCINT8-14: Off
	// Interrupt on any change on pins PCINT16-23: Off
	EICRA=(0<<ISC11) | (0<<ISC10) | (0<<ISC01) | (0<<ISC00);
	EIMSK=(0<<INT1) | (1<<INT0); //Включаем внешние прерывания
	EIFR=(0<<INTF1) | (0<<INTF0);
	PCICR=(0<<PCIE2) | (0<<PCIE1) | (0<<PCIE0);
	PCMSK0=(0<<PCINT7) | (0<<PCINT6) | (0<<PCINT5) | (0<<PCINT4) | (0<<PCINT3) | (0<<PCINT2) | (0<<PCINT1) | (0<<PCINT0);
	PCIFR=(0<<PCIF2) | (0<<PCIF1) | (1<<PCIF0);	
}


// Read the AD conversion result
unsigned int read_adc(unsigned char adc_input)
{
	ADMUX=adc_input | ADC_VREF_TYPE;
	// Delay needed for the stabilization of the ADC input voltage
	_delay_us(10);
	// Start the AD conversion
	ADCSRA|=(1<<ADSC);
	// Wait for the AD conversion to complete
	while ((ADCSRA & (1<<ADIF))==0);
	ADCSRA|=(1<<ADIF);
	return ADCW;
}

unsigned int checkBat (void) //Проверка основной батареи
{
	while(read_adc(0)*1.055*25.12==0);
	return read_adc(0)*1.055*25.12;
}

unsigned int checkResBat (void) //Проверка резервной батареи
{
	return read_adc(7)*1.055*4.02;
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
	while ( !( UCSR0A & (1<<UDRE0)) )	;
	/* Put data into buffer, sends the data */
	UDR0 = data;
}

void U0_Puts (char S[])
{
	unsigned char length = strlen(S);
	for (unsigned char i=0;i<length;i++) USART_Transmit(S[i]);
	//USART_Transmit('\r');
	//USART_Transmit('\n');
}

void InitDS1337 (void)
{
	//DS1337_WriteRegister(0x0E,0x00);
	DS1337_ClearBit(0x0E,INTCN);
	DS1337_ClearBit(0x0E,EOSC);
	DS1337_SetBit(0x0E,RS1);
	DS1337_SetBit(0x0E,RS2);
	
	
	//Настраиваем параметры будильника
	DS1337_ClearBit(0x07,A1M1); 
	DS1337_ClearBit(0x08,A1M2);
	DS1337_ClearBit(0x09,A1M3);
	DS1337_ClearBit(0x0A,A1M4);
	
	
	//Сбрасываем флаг прерывания
	//DS1337_ClearBit(0x0f,A1F);
}

void goSleep (unsigned char mode)
{
	switch (mode)
	{
		case Down :
		{
				if (eeprom_read_byte(&SLP) == 0) //Если флаг обнулен
				{
					U0_Puts("\r\n Go sleep\r\n");
					eeprom_write_byte(&SLP,1); //Устанавливаем
					while(!eeprom_is_ready()); 
					//Отключаем TWI
					TWI_Enable(false);
					//Отключаем прерывание  PCINT0-7
					PCICR=(0<<PCIE2) | (0<<PCIE1) | (0<<PCIE0);					
					set_sleep_mode(SLEEP_MODE_PWR_DOWN); // если спать - то на полную
					sleep_enable(); // разрешаем сон					
				}			
		}
		break;
		case Up:
		{
			if (eeprom_read_byte(&SLP) == 1) //Если флаг установлен
			{				
				U0_Puts("\r\n Up\r\n");
				eeprom_write_byte(&SLP,0); //Устанавливаем 
				while(!eeprom_is_ready());
				//Включаем TWI
				TWI_Enable(true);
				//Включаем прерывание  PCINT0-7
				PCICR=(0<<PCIE2) | (0<<PCIE1) | (1<<PCIE0);
				sleep_disable();
			}
		}
		break;
	}
	
}

unsigned char ControlSumm (unsigned char *Mass, unsigned int length)
{
	unsigned int output;
	output=*(Mass);
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

void deletePacket(unsigned int endPos)
{
	for (unsigned int i=endPos;i<bufSize;i++)
	{
		inputData[i-endPos]=inputData[i];
		inputData[i]=0;
	}
	inputDataCount = inputDataCount - endPos;
}

void clearInputData(void)
{
	for (unsigned int i=0;i<inputDataCount;i++) inputData[i]=0;
	Pos=-1;
	lenPacket=0;
	inputDataCount = 0;
}

void deleteHeader(unsigned char Pos)
{
	for (unsigned int i=Pos;i<bufSize-1;i++) inputData[i]=inputData[i+1];
	inputData[bufSize]=0;
	inputDataCount --;
}

void writeStructDeviceInfo(void)
{
	//Запись версий и типа устройства
	eeprom_update_byte(&deviceType_e,deviceType);
	eeprom_update_byte(&firmwareVersion_e,firmwareVersion);
	eeprom_update_byte(&haedwareVersion_e,haedwareVersion);
	
	eeprom_update_block(&serialNumber,&serialNumber_e,5);
	
	//Запись параметров батареи
	eeprom_update_word(&VBatMin_e,VBatMin);
	
	eeprom_update_word(&MinBat1_e,MinBat1);
	eeprom_update_word(&MaxBat1_e,MaxBat1);
	eeprom_update_word(&MinBat2_e,MinBat2);
	eeprom_update_word(&MaxBat2_e,MaxBat2);	
	
	//Запись дополнительных параметров	
	eeprom_update_byte(&isRun_e,isRun);
	eeprom_update_byte(&currentPointNumber_e,currentPointNumber);
	eeprom_update_byte(&currentPower_e,currentPower);
	eeprom_update_byte(&currentPWMMin_e,currentPWMMin);
	eeprom_update_byte(&currentPWMMax_e,currentPWMMax);
	eeprom_update_byte(&currentExtPower_e,currentExtPower);
	eeprom_update_byte(&currentBPower_e,currentBPower);
	
	//Запись параметров режима работы передатчика
	eeprom_update_byte(&countSignal_e,countSignal);
	eeprom_update_byte(&countSignalPause_e,countSignalPause);
	eeprom_update_byte(&currentTransmitMode_e,currentTransmitMode);
	
	eeprom_update_block(&currentMSequence,&currentMSequence_e,4); //Сохранаем текущую М-последовательность
	eeprom_update_byte(&currentMSequenceLen_e,currentMSequenceLen); //Сохраняем длину текущей М-последовательности
}

void readStructDeviceInfo(void) //Информация об устройстве
{
		//Чтение версий и типа устройства
		deviceType = eeprom_read_byte(&deviceType_e);
		firmwareVersion = eeprom_read_byte(&firmwareVersion_e);
		firmwareVersion = MaxFirmwareVersion;
		haedwareVersion = eeprom_read_byte(&haedwareVersion_e);
		
		eeprom_read_block(&serialNumber,&serialNumber_e,5);
		
		
		
		//Чтение параметров батареи
		MinBat1 = eeprom_read_word(&MinBat1_e);
		if ((MinBat1<currentMinLeveBat1)||(MinBat1>currentMaxLeveBat1)) MinBat1 = currentMinLeveBat1;
		MaxBat1 = eeprom_read_word(&MaxBat1_e);
		if ((MaxBat1<currentMinLeveBat1)||(MaxBat1>currentMaxLeveBat1)) MaxBat1 = currentMaxLeveBat1;
		MinBat2 = eeprom_read_word(&MinBat2_e);
		if ((MinBat2<currentMinLeveBat2)||(MinBat2>currentMaxLeveBat2)) MinBat2 = currentMinLeveBat2;
		MaxBat2 = eeprom_read_word(&MaxBat2_e);
		if ((MaxBat2<currentMinLeveBat2)||(MaxBat2>currentMaxLeveBat2)) MaxBat2 = currentMaxLeveBat2;
		
		VBatMin = eeprom_read_word(&VBatMin_e);
		if ((VBatMin<MinBat2)||(VBatMin>MaxBat2)) VBatMin = MinBat2;
		
		//Чтение дополнительных параметров
		isRun = eeprom_read_byte(&isRun_e);
		currentPointNumber = eeprom_read_byte(&currentPointNumber_e);
		if (currentPointNumber>PointsCount) currentPointNumber = 0;
		currentPWMMin = eeprom_read_byte(&currentPWMMin_e);
		if (currentPWMMin>0xFA) currentPWMMin = 1;
		currentPWMMax = eeprom_read_byte(&currentPWMMax_e);
		currentPower = eeprom_read_byte(&currentPower_e);
		PWM_SetPower((currentPWMMax/100.0) * (currentPower));
		//currentExtPower = eeprom_read_byte(&currentExtPower_e);
		currentBPower = eeprom_read_byte(&currentBPower_e);
		setPowerExt(eeprom_read_byte(&currentExtPower_e));
		
		//Чтение параметров режима работы передатчика
		countSignal = eeprom_read_byte(&countSignal_e);
		if (countSignal>0x7D) countSignal = 0x7D;
		countSignalPause = eeprom_read_byte(&countSignalPause_e);
		if (countSignalPause>0x7D) countSignalPause = 0x7D;
		currentTransmitMode = eeprom_read_byte(&currentTransmitMode_e);
		if (currentTransmitMode>MaxTransmitMode) currentTransmitMode = 0x00;
		PWM_SetMode(currentTransmitMode,countSignalPause,countSignal);
		
		eeprom_read_block(&currentMSequence,&currentMSequence_e,4); //Читаем текущую М-последовательность
		currentMSequenceLen = eeprom_read_byte(&currentMSequenceLen_e); //Читаем длину текущей М-последовательности
		
		PWM_SetMSequence(currentMSequence,currentMSequenceLen);
}

void readPoints(void)
{
		eeprom_read_block((void *)&hours, (const void *)&hours_e,PointsCount);
		eeprom_read_block((void *)&minutes, (const void *)&minutes_e,PointsCount);
		eeprom_read_block((void *)&seconds, (const void *)&seconds_e,PointsCount);
		eeprom_read_block((void *)&days, (const void *)&days_e,PointsCount);
		eeprom_read_block((void *)&month, (const void *)&month_e,PointsCount);
		eeprom_read_block((void *)&year, (const void *)&year_e,PointsCount);
		eeprom_read_block((void *)&config, (const void *)&config_e,PointsCount);
		eeprom_read_block((void *)&power, (const void *)&power_e,PointsCount);
		eeprom_read_block((void *)&VBat, (const void *)&VBat_e,2*PointsCount);

		bool write = false;
	for (unsigned char i=0;i<PointsCount;i++)
	{		
		if ((hours[i]==255)||(minutes[i]==255)||(seconds[i]==255)||(month[i]==255)||(year[i]==255))
		{
			write = true;
			hours[i]=0;
			minutes[i]=0;
			seconds[i]=0;
			days[i]=1;
			month[i]=1;
			year[i]=0;
			config[i]=0x7f;
			power[i]=50;
			VBat[i]=0;
		}
	}
	if (write) writePoints();
}

void writePoints(void)
{
	eeprom_update_block(&hours,&hours_e, PointsCount);
	eeprom_update_block(&minutes,&minutes_e, PointsCount);
	eeprom_update_block(&seconds,&seconds_e, PointsCount);
	eeprom_update_block(&days,&days_e, PointsCount);
	eeprom_update_block(&month,&month_e, PointsCount);
	eeprom_update_block(&year,&year_e, PointsCount);
	eeprom_update_block(&config,&config_e, PointsCount);
	eeprom_update_block(&power,&power_e, PointsCount);
	eeprom_update_block(&VBat,&VBat_e, 2*PointsCount);
}

uint16_t getNextPoint(void) //Найти следующую точку
{
	uint8_t Output = 255;
	Time_ currentTime;
	Date_ currentDate;
	bool result=false;
	DS1337_GetTime(&currentTime,DS1337_24H); //Получаем текущее время
	DS1337_GetDate(&currentDate); //Получаем текущую дату
	for (uint8_t i=currentPointNumber;i<PointsCount;i++) //Перебираем точки начиная с последующей за текущей точки
	{
		if (config[i] & 0x80) //Проверяем активна ли точка
			if (year[i]>currentDate.Year) {Output = i; break;}
			else if (year[i]==currentDate.Year) 
			if (month[i]>currentDate.Month) {Output = i; break;}
			else  if (month[i]==currentDate.Month)
			if (days[i]>currentDate.Day) {Output = i; break;}
			else if (days[i]==currentDate.Day)
			if (hours[i]>currentTime.Hours) {Output = i; break;}
			else if (hours[i]==currentTime.Hours)
			if (minutes[i]>currentTime.Minutes) {Output = i; break;}
			else if (minutes[i]==currentTime.Minutes) 
			if (seconds[i]+10>currentTime.Seconds) {Output = i; break;}
	}
	
	return Output;
}

void sendPowerState()
{
	outputData[0] = Header;
	outputData[1] = 0x05;
	outputData[2] = 0x00;
	outputData[3] = 0x08;
	outputData[4] = currentExtPower;
	outputData[5] = ControlSumm(&outputData,5);
	
	for (unsigned char i=0;i<6;i++) USART_Transmit(outputData[i]);
}

void setPowerExt(uint8_t Power)
{
	currentExtPower = Power;
	switch (deviceType)
	{
		case typeSealer: if (currentExtPower) PowerOn; else PowerOff;
		break;
		case typeSearch: if (currentExtPower) PWM_Start(); else PWM_Stop();
		break;		
	}
	
	
	outputData[0] = Header;
	outputData[1] = 0x05;
	outputData[2] = 0x00;
	outputData[3] = 0x08;
	outputData[4] = currentExtPower;
	outputData[5] = ControlSumm(&outputData,5);
		
	for (unsigned char i=0;i<6;i++) USART_Transmit(outputData[i]);
}

void setBpower(uint8_t Power)
{
	currentBPower = Power;
	switch (deviceType)
	{
		case typeSealer: if (currentBPower) PORTD |= (1<<PORTD4); else PORTD  &= ~(1<<PORTD4);
		break;
		case typeSearch: if (currentBPower)  PORTD  &= ~(1<<PORTD3); else PORTD |= (1<<PORTD3);
		break;
	}
}

void handingMissingPoints(uint8_t *startPoint,uint8_t *endPoint) //Обработка пропущенных точек
{
	if (*endPoint!=255)
	{
		for (uint8_t i=*startPoint;i<*endPoint;i++)
			if (config[i] & 0x80) //Проверяем активна ли точка
			{
				config[i] = config[i] & 0x7F; //Сбрасываем флаг активности точки
				currentPower = power[i];
				PWM_SetPower((currentPWMMax/100.0) * (currentPower));
				//currentExtPower = config[i] & 0x01;
				setPowerExt(config[i] & 0x01);
				VBat[i] = checkBat();
				currentBPower = config[i] & 0x02;
			}
		writePoints();
	}
}

void executeCurrentPoint(void) //Выполнить текущую точку
{
	//Выполняем условия текущей точки
	unsigned int ptr = 0xFF00;
	setPowerExt(config[currentPointNumber] & 0x01); //Устанавливаем режим внешнего питания
	VBat[currentPointNumber] = checkBat(); //Сохраняем текущее состояние батареи

	setBpower(config[currentPointNumber] & 0x02); //Устанавливаем состояние питания блютуз модуля

	if (deviceType == typeSearch) //Если тип устройства поисковое то устанавливаем текущую мощность
	{
		currentPower = power[currentPointNumber];
		PWM_SetPower((currentPWMMax/100.0) * (currentPower));
	}
			
	writePoints();
	writeStructDeviceInfo();
			
	packet0x17(); //Отправляем текущее состояние
	packet0x09(); //Отправляем время будильника
	sendPowerState(); //Отправляем информацию о внешнем питании
	packet0x0B((unsigned char*)(&ptr)); //Отправляем точки
}

void packet0x01(void) //Отправка информации об устройстве
{
	outputData[0] = Header;
	outputData[1] = 0x0c;
	outputData[2] = 0x00;
	outputData[3] = 0x01;
	outputData[4] = deviceType;
	outputData[5] = firmwareVersion;
	outputData[6] = haedwareVersion;

	for (unsigned char i=0;i<5;i++) outputData[7+i] = serialNumber[i];
	
	outputData[12] = ControlSumm(&outputData,12);
	
	for (unsigned char i=0;i<13;i++) USART_Transmit(outputData[i]);
	
}

void packet0x02(void) //Статус батарей
{
	bat1 = checkResBat();
	bat1 = checkResBat();
	bat2 = checkBat();
	bat2 = checkBat();
	
	outputData[0] = Header;
	outputData[1] = 0x08;
	outputData[2] = 0x00;
	outputData[3] = 0x02;
	outputData[4] = bat1;
	outputData[5] = bat1>>8;
	outputData[6] = bat2;
	outputData[7] = bat2>>8;
	outputData[8] = ControlSumm(&outputData,8);
	
	for (unsigned char i=0;i<9;i++) USART_Transmit(outputData[i]);
}

void packet0x03(void) //Время устройства
{
	DS1337_GetTime(&time,DS1337_24H);
	outputData[0] = Header;
	outputData[1] = 0x07;
	outputData[2] = 0x00;
	outputData[3] = 0x03;
	outputData[4] = time.Hours;
	outputData[5] = time.Minutes;
	outputData[6] = time.Seconds;
	outputData[7] = ControlSumm(&outputData,7);
	
	for (unsigned char i=0;i<8;i++) USART_Transmit(outputData[i]);
}

void packet0x04(void) //Дата устройства
{
	DS1337_GetDate(&date);
	outputData[0] = Header;
	outputData[1] = 0x08;
	outputData[2] = 0x00;
	outputData[3] = 0x04;
	outputData[4] = date.Day;
	outputData[5] = date.Month;
	outputData[6] = date.Year;
	outputData[7] = date.DayW;
	outputData[8] = ControlSumm(&outputData,8);
	
	for (unsigned char i=0;i<9;i++) USART_Transmit(outputData[i]);
}

void packet0x05(unsigned char *input) //Установка времени
{
	unsigned char result;

	time.Hours	  = *(input+1);
	time.Minutes  = *(input+2);
	time.Seconds  = *(input+3);

	if ((time.Hours<24)&&(time.Minutes<60)&&(time.Seconds<60))
	{
		DS1337_SetTime(&time);
		result = 0x01;
	} else result =0x00;

	
	outputData[0] = Header;
	outputData[1] = 0x05;
	outputData[2] = 0x00;
	outputData[3] = 0x05;
	outputData[4] = result;
	outputData[5] = ControlSumm(&outputData,5);
		
	for (unsigned char i=0;i<6;i++) USART_Transmit(outputData[i]);
}

void packet0x06(unsigned char *input) //Установка даты
{
	unsigned char result;

	date.Day	= *(input+1);
	date.Month  = *(input+2);
	date.Year   = *(input+3);
	date.DayW   = *(input+4);

	if ((date.Day<32)&&(date.Month<13)&&(date.DayW<8))
	{
		DS1337_SetDate(&date);
		result = 0x01;
	} else result =0x00;

	
	outputData[0] = Header;
	outputData[1] = 0x05;
	outputData[2] = 0x00;
	outputData[3] = 0x06;
	outputData[4] = result;
	outputData[5] = ControlSumm(&outputData,5);
		
	for (unsigned char i=0;i<6;i++) USART_Transmit(outputData[i]);
}

void packet0x07(unsigned char *input) //Установка будильника
{
	unsigned char result;

	time.Hours	  = *(input+1);
	time.Minutes  = *(input+2);
	time.Seconds  = *(input+3);
	unsigned char Day = *(input+4);
	
	if ((time.Hours<24)&&(time.Minutes<60)&&(time.Seconds<60))
	{
		DS1337_SetAlarm1(&time,Day);
		result = 0x01;
	} else result =0x00;

	
	outputData[0] = Header;
	outputData[1] = 0x05;
	outputData[2] = 0x00;
	outputData[3] = 0x07;
	outputData[4] = result;
	outputData[5] = ControlSumm(&outputData,5);
	
	for (unsigned char i=0;i<6;i++) USART_Transmit(outputData[i]);
}

void packet0x08(unsigned char *input) //Управление внешним питанием
{
	unsigned char result;
	
	if (*(input+1))
	{
		//PowerOn;
		result = 0x01;
		setPowerExt(result);
		//currentExtPower=0x01;
	} else 
	{
		//PowerOff;
		result =0x00;
		setPowerExt(result);
		//currentExtPower=0x00;
	}

		
	outputData[0] = Header;
	outputData[1] = 0x05;
	outputData[2] = 0x00;
	outputData[3] = 0x08;
	outputData[4] = result;
	outputData[5] = ControlSumm(&outputData,5);
	
	for (unsigned char i=0;i<6;i++) USART_Transmit(outputData[i]);	
	
	writeStructDeviceInfo();
}

void packet0x09(void) //Время будильника
{
	unsigned char day;
	DS1337_GetAlarm1(&time,&day);
	outputData[0] = Header;
	outputData[1] = 0x08;
	outputData[2] = 0x00;
	outputData[3] = 0x09;
	outputData[4] = time.Hours;
	outputData[5] = time.Minutes;
	outputData[6] = time.Seconds;
	outputData[7] = day;
	outputData[8] = ControlSumm(&outputData,8);
	
	for (unsigned char i=0;i<9;i++) USART_Transmit(outputData[i]);
}

void packet0x0A(void) //Перезагрузка устройства
{
	outputData[0] = Header;
	outputData[1] = 0x05;
	outputData[2] = 0x00;
	outputData[3] = 0x0A;
	outputData[4] = 0x01;
	outputData[5] = ControlSumm(&outputData,5);
	
	for (unsigned char i=0;i<6;i++) USART_Transmit(outputData[i]);
	funcptr();
}

void packet0x0B(unsigned char *input) //Чтение точки
{
	if (*(input+1)==255)
	{
		outputData[0] = Header;
		outputData[1] = 5+sizeOfStructPoint*PointsCount;
		outputData[2] = 0x00;
		outputData[3] = 0x0B;
		outputData[4] = 0xFF;
		
		for (unsigned char i=0;i<PointsCount;i++)
		{
			outputData[5+sizeOfStructPoint*i] = hours[i];
			outputData[6+sizeOfStructPoint*i] = minutes[i];
			outputData[7+sizeOfStructPoint*i] = seconds[i];
			outputData[8+sizeOfStructPoint*i] = days[i];
			outputData[9+sizeOfStructPoint*i] = month[i];
			outputData[10+sizeOfStructPoint*i] = year[i];
			outputData[11+sizeOfStructPoint*i] = config[i];
			outputData[12+sizeOfStructPoint*i] = power[i];
			outputData[13+sizeOfStructPoint*i] = *((unsigned char*)(&VBat[i]));
			outputData[14+sizeOfStructPoint*i] = *(((unsigned char*)(&VBat[i]))+1);
		}
		outputData[5+sizeOfStructPoint*PointsCount] = ControlSumm(&outputData,5+sizeOfStructPoint*PointsCount);
		for (unsigned char i=0;i<5+sizeOfStructPoint*PointsCount+1;i++) USART_Transmit(outputData[i]);		
	} else
	{
		outputData[0] = Header;
		outputData[1] = 0x0C;
		outputData[2] = 0x00;
		outputData[3] = 0x0B;
		outputData[4] = *(input+1);
		outputData[5] = hours[*(input+1)];
		outputData[6] = minutes[*(input+1)];
		outputData[7] = seconds[*(input+1)];
		outputData[8] = days[*(input+1)];
		outputData[9] = month[*(input+1)];
		outputData[10] = year[*(input+1)];
		outputData[11] = config[*(input+1)];
		outputData[12] = power[*(input+1)];
		outputData[13] = *((unsigned char*)(&VBat[*(input+1)]));
		outputData[14] = *((unsigned char*)((&VBat[*(input+1)])+1));
		
		outputData[15] = ControlSumm(&outputData,15);
		for (unsigned char i=0;i<16;i++) USART_Transmit(outputData[i]);
	}
}

void packet0x0C(unsigned char *input) //Запись точки
{
	bool output = true;
	if (*(input+1)==1)
	{
		hours[*(input+2)] = *(input+2);
		minutes[*(input+2)] = *(input+3);
		seconds[*(input+2)] = *(input+4);
		days[*(input+2)] = *(input+5);
		month[*(input+2)] = *(input+6);
		year[*(input+2)] = *(input+7);
		config[*(input+2)] = *(input+8);
		power[*(input+2)] = *(input+9);
		*((unsigned char *)(&VBat[*(input+2)])) = *(input+10);
		*((unsigned char *)((&VBat[*(input+2)]))+1) = *(input+11);
	} else
	{
		for (unsigned char i=0;i<*(input+1);i++)
		{
			unsigned char j = i*sizeOfStructPoint;
			hours[i] = *(input+2+j);
			minutes[i] = *(input+3+j);
			seconds[i] = *(input+4+j);
			days[i] = *(input+5+j);
			month[i] = *(input+6+j);
			year[i] = *(input+7+j);
			config[i] = *(input+8+j);
			power[i] = *(input+9+j);
			*((unsigned char *)(&VBat[i])) = *(input+10+j);
			*((unsigned char *)((&VBat[i]))+1) = *(input+11+j);
		}
	}
	writePoints();
	
	if (*(input+1)<PointsCount)
	{
		outputData[0] = Header;
		outputData[1] = 0x06;
		outputData[2] = 0x00;
		outputData[3] = 0x0C;
		outputData[4] = output;
		outputData[5] = *(input+2);
		outputData[6] = ControlSumm(&outputData,6);
		for (unsigned char i=0;i<7;i++) USART_Transmit(outputData[i]);
	} else
	{
		outputData[0] = Header;
		outputData[1] = 0x06;
		outputData[2] = 0x00;
		outputData[3] = 0x0C;
		outputData[4] = output;
		outputData[5] = 0x7f;
		outputData[6] = ControlSumm(&outputData,6);
		for (unsigned char i=0;i<7;i++) USART_Transmit(outputData[i]);
	}
	
}

void packet0x0D(void) //Отправить уровень отключения батареи
{
	outputData[0] = Header;
	outputData[1] = 0x06;
	outputData[2] = 0x00;
	outputData[3] = 0x0D;
	outputData[4] = *((unsigned char*)(&VBatMin));
	outputData[5] = *(((unsigned char*)(&VBatMin))+1);
	outputData[6] = ControlSumm(&outputData,6);
	for (unsigned char i=0;i<7;i++) USART_Transmit(outputData[i]);	
}

void packet0x0E(unsigned char *input) //Установить уровень отключения батареи
{
	bool result =true;
	unsigned int Buf;
	*((unsigned char*)(&Buf))=*(input+1);
	*(((unsigned char*)(&Buf))+1)=*(input+2);
	
	if (Buf<30000)
	{
		VBatMin = Buf;
		writeStructDeviceInfo();
	} else result = false;
	
	outputData[0] = Header;
	outputData[1] = 0x05;
	outputData[2] = 0x00;
	outputData[3] = 0x0E;
	outputData[4] = result;
	outputData[5] = ControlSumm(&outputData,5);
	
	for (unsigned char i=0;i<6;i++) USART_Transmit(outputData[i]);
}

void packet0x0F(unsigned char *input) //Активировать режим сна
{
	bool result = true;
	
	outputData[0] = Header;
	outputData[1] = 0x05;
	outputData[2] = 0x00;
	outputData[3] = 0x0F;
	outputData[4] = result;
	outputData[5] = ControlSumm(&outputData,5);
	
	for (unsigned char i=0;i<6;i++) USART_Transmit(outputData[i]);
	
	goSleep(Down);
}

void packet0x10(void) //Прочитать все
{
	PWM_Stop();
		
	packet0x03 ();
	packet0x04 ();
	packet0x09 ();
	packet0x0D ();
	packet0x12 ();
	packet0x14 ();
	packet0x02 ();
	packet0x17 ();
	packet0x1A ();
	packet0x1C ();	
	packet0x1E ();
	packet0x22 ();
	packet0x24 ();
	packet0x26 ();
	sendPowerState();
	
	for (unsigned char i=0;i<6;i++) USART_Transmit(outputData[i]);
	
	int v = 0xffff;
	packet0x0B(&v);
	
	outputData[0] = Header;
	outputData[1] = 0x05;
	outputData[2] = 0x00;
	outputData[3] = 0x10;
	outputData[4] = 0x01;
	outputData[5] = ControlSumm(&outputData,5);
	for (unsigned char i=0;i<6;i++) USART_Transmit(outputData[i]);
	
	if (currentExtPower) PWM_Start();
} 

void packet0x11 (void) //Срабатывание будильника
{
	outputData[0] = Header;
	outputData[1] = 0x05;
	outputData[2] = 0x00;
	outputData[3] = 0x11;
	outputData[4] = 0x01;
	outputData[5] = ControlSumm(&outputData,5);
		
	for (unsigned char i=0;i<6;i++) USART_Transmit(outputData[i]);
}

void packet0x12(void) //Прочитать уровенb первой батареи
{
	outputData[0] = Header;
	outputData[1] = 0x08;
	outputData[2] = 0x00;
	outputData[3] = 0x12;
	outputData[4] = *((unsigned char *)(&MinBat1));
	outputData[5] = *(((unsigned char *)(&MinBat1))+1);
	outputData[6] = *((unsigned char *)(&MaxBat1));
	outputData[7] = *(((unsigned char *)(&MaxBat1))+1);
	outputData[8] = ControlSumm(&outputData,8);
	
	for (unsigned char i=0;i<9;i++) USART_Transmit(outputData[i]);
}

void packet0x13(unsigned char *input) //Установить минимальный уровень первой батареи
{
	bool result=true;

	*((unsigned char *)(&MinBat1)) = *(input+1);;
	*(((unsigned char *)(&MinBat1))+1) = *(input+2);
	*((unsigned char *)(&MaxBat1)) = *(input+3);
	*(((unsigned char *)(&MaxBat1))+1) = *(input+4);
	
	writeStructDeviceInfo();
	
	outputData[0] = Header;
	outputData[1] = 0x05;
	outputData[2] = 0x00;
	outputData[3] = 0x13;
	outputData[4] = result;
	outputData[5] = ControlSumm(&outputData,5);
	
	for (unsigned char i=0;i<6;i++) USART_Transmit(outputData[i]);
}

void packet0x14(void) //Прочитать минимальный уровень второй батареи
{
	outputData[0] = Header;
	outputData[1] = 0x08;
	outputData[2] = 0x00;
	outputData[3] = 0x14;
	outputData[4] = *((unsigned char *)(&MinBat2));
	outputData[5] = *(((unsigned char *)(&MinBat2))+1);
	outputData[6] = *((unsigned char *)(&MaxBat2));
	outputData[7] = *(((unsigned char *)(&MaxBat2))+1);
	outputData[8] = ControlSumm(&outputData,8);
	
	for (unsigned char i=0;i<9;i++) USART_Transmit(outputData[i]);
}

void packet0x15(unsigned char *input) //Установить минимальный уровень второй батареи
{
	bool result=true;

	*((unsigned char *)(&MinBat2)) = *(input+1);;
	*(((unsigned char *)(&MinBat2))+1) = *(input+2);
	*((unsigned char *)(&MaxBat2)) = *(input+3);
	*(((unsigned char *)(&MaxBat2))+1) = *(input+4);
	
	writeStructDeviceInfo();
	
	outputData[0] = Header;
	outputData[1] = 0x05;
	outputData[2] = 0x00;
	outputData[3] = 0x15;
	outputData[4] = result;
	outputData[5] = ControlSumm(&outputData,5);
	
	for (unsigned char i=0;i<6;i++) USART_Transmit(outputData[i]);
}

void packet0x16(unsigned char *input) //Записать информацию об устройстве
{
	bool outputDeviceType = false;
	bool outputFirmwareVersion = false;
	bool outputHardwareVersion = false;
	bool outputSerialNumber = false;
	
	if ((deviceType <0x01)||(deviceType>MaxDeviceType))
	{
		deviceType = *(input+1);
		outputDeviceType = true;
	}
	
	if ((firmwareVersion < 0x01)||(firmwareVersion>MaxFirmwareVersion))
	{
		firmwareVersion = *(input+2);
		outputFirmwareVersion = true;
	}
	
	if ((haedwareVersion < 0x01)||(haedwareVersion>MaxHardwareVersion))
	{
		haedwareVersion = *(input+3);
		outputHardwareVersion = true;
	}
	
	bool checkSerialNumber =true;
	for (unsigned char i =0;i<5;i++)
		if ((serialNumber[i]!=0xff)&&(serialNumber[i]!=0x00)) checkSerialNumber=false;
		
	if (checkSerialNumber)
	{
		for (unsigned char i=4;i<9;i++) serialNumber[i-4] = *(input+i);
		outputSerialNumber = true;
	}
	writeStructDeviceInfo();
	
	outputData[0] = Header;
	outputData[1] = 0x08;
	outputData[2] = 0x00;
	outputData[3] = 0x16;
	outputData[4] = outputDeviceType;
	outputData[5] = outputFirmwareVersion;
	outputData[6] = outputHardwareVersion;
	outputData[7] = outputSerialNumber;
	outputData[8] = ControlSumm(&outputData,8);
	
	for (unsigned char i=0;i<9;i++) USART_Transmit(outputData[i]);
}

void packet0x17(void) //Получить текущее состояние
{
	outputData[0] = Header;
	outputData[1] = 0x06;
	outputData[2] = 0x00;
	outputData[3] = 0x17;
	outputData[4] = isRun;
	outputData[5] = currentPointNumber;
	outputData[6] = ControlSumm(&outputData,6);
	
	for (unsigned char i=0;i<7;i++) USART_Transmit(outputData[i]);
}

void packet0x18(unsigned char *input) //Запустить алгоритм
{
	bool Output = false;
	uint8_t bufPointNumber;
	if (!isRun) //Проверяем не запущен ли алгоритм ранее
	{
		currentPointNumber = *(input+1); //Устанавливаем полученную точку как текущую
		bufPointNumber = getNextPoint(); //Проверяем текущую точку, если нет то ищем следующую подходящую
		if (bufPointNumber<PointsCount) //Если подхожящая точка найдена - запускаем лагоритм
		{
			isRun = true;
			Output = true;
			currentPointNumber = bufPointNumber;
		}		
	}
	
	
	
	writeStructDeviceInfo();
	
	outputData[0] = Header;
	outputData[1] = 0x05;
	outputData[2] = 0x00;
	outputData[3] = 0x18;
	outputData[4] = Output;
	outputData[5] = ControlSumm(&outputData,5);
		
	for (unsigned char i=0;i<6;i++) USART_Transmit(outputData[i]);
	
	if ((bufPointNumber<PointsCount)&&(isRun)) //Если устройство запушено и есть подходящая точка
	{		
		Time_ currentTime;
		currentTime.Hours = hours[currentPointNumber];
		currentTime.Minutes = minutes[currentPointNumber];
		currentTime.Seconds = seconds[currentPointNumber];
		DS1337_SetAlarm1(&currentTime,days[currentPointNumber]); //Устанавливаем будильник
		packet0x09(); //Отправляем время будильника
		
		writeStructDeviceInfo();
		
		packet0x17(); //Отправляем текущее состояние
		packet0x09(); //Отправляем время будильника		
	}
}

void packet0x19() //Остановить алгортм
{
	bool Output = false;
		
	if (isRun)
	{
		isRun = false;
		Output = true;
	}
	
	writeStructDeviceInfo();
	
	outputData[0] = Header;
	outputData[1] = 0x05;
	outputData[2] = 0x00;
	outputData[3] = 0x19;
	outputData[4] = Output;
	outputData[5] = ControlSumm(&outputData,5);
	
	for (unsigned char i=0;i<6;i++) USART_Transmit(outputData[i]);
}

void packet0x1A(void) //Прочитать текущую мощность передатчика
{
	outputData[0] = Header;
	outputData[1] = 0x05;
	outputData[2] = 0x00;
	outputData[3] = 0x1A;
	outputData[4] = currentPower;
	outputData[5] = ControlSumm(&outputData,5);
	
	for (unsigned char i=0;i<6;i++) USART_Transmit(outputData[i]);
}

void packet0x1B(unsigned char *input) //Установить текущую мощность передатчика
{
	bool Output = false;
	
	if ((*(input+1)>=0)&&(*(input+1)<=100)&&(deviceType == typeSearch))
	{
		Output = true;
		currentPower = *(input+1);
		writeStructDeviceInfo();
		PWM_SetPower((currentPWMMax/100.0) * (*(input+1)));
	}
	
	outputData[0] = Header;
	outputData[1] = 0x05;
	outputData[2] = 0x00;
	outputData[3] = 0x1B;
	outputData[4] = Output;
	outputData[5] = ControlSumm(&outputData,5);
	
	for (unsigned char i=0;i<6;i++) USART_Transmit(outputData[i]);	
}

void packet0x1C(void) //Прочитать параметры ШИМ
{
	outputData[0] = Header;
	outputData[1] = 0x06;
	outputData[2] = 0x00;
	outputData[3] = 0x1C;
	outputData[4] = currentPWMMin;
	outputData[5] = currentPWMMax;
	outputData[6] = ControlSumm(&outputData,6);
	
	for (unsigned char i=0;i<7;i++) USART_Transmit(outputData[i]);
}

void packet0x1D(unsigned char *input) //Установить параметры ШИМ
{

	currentPWMMin = *(input+1);
	currentPWMMax = *(input+2);
	
	writeStructDeviceInfo();
	
	outputData[0] = Header;
	outputData[1] = 0x05;
	outputData[2] = 0x00;
	outputData[3] = 0x1D;
	outputData[4] = 0x01;
	outputData[5] = ControlSumm(&outputData,5);
	
	for (unsigned char i=0;i<6;i++) USART_Transmit(outputData[i]);
}

void packet0x1E(void) //Прочитать регистры RTC
{
	outputData[0] = Header;
	outputData[1] = 0x14;
	outputData[2] = 0x00;
	outputData[3] = 0x1E;
	outputData[4] = DS1337_ReadRegister(0x00);
	outputData[5] = DS1337_ReadRegister(0x01);
	outputData[6] = DS1337_ReadRegister(0x02);
	outputData[7] = DS1337_ReadRegister(0x03);
	outputData[8] = DS1337_ReadRegister(0x04);
	outputData[9] = DS1337_ReadRegister(0x05);
	outputData[10] = DS1337_ReadRegister(0x06);
	outputData[11] = DS1337_ReadRegister(0x07);
	outputData[12] = DS1337_ReadRegister(0x08);
	outputData[13] = DS1337_ReadRegister(0x09);
	outputData[14] = DS1337_ReadRegister(0x0A);
	outputData[15] = DS1337_ReadRegister(0x0B);
	outputData[16] = DS1337_ReadRegister(0x0C);
	outputData[17] = DS1337_ReadRegister(0x0D);
	outputData[18] = DS1337_ReadRegister(0x0E);
	outputData[19] = DS1337_ReadRegister(0x0F);
	outputData[20] = ControlSumm(&outputData,20);
	
	for (unsigned char i=0;i<21;i++) USART_Transmit(outputData[i]);
}

void packet0x1F(unsigned char *input) //Записать регистры RTC
{
	DS1337_WriteRegister(0x00,*(input+1));
	DS1337_WriteRegister(0x01,*(input+2));
	DS1337_WriteRegister(0x02,*(input+3));
	DS1337_WriteRegister(0x03,*(input+4));
	DS1337_WriteRegister(0x04,*(input+5));
	DS1337_WriteRegister(0x05,*(input+6));
	DS1337_WriteRegister(0x06,*(input+7));
	DS1337_WriteRegister(0x07,*(input+8));
	DS1337_WriteRegister(0x08,*(input+9));
	DS1337_WriteRegister(0x09,*(input+10));
	DS1337_WriteRegister(0x0A,*(input+11));
	DS1337_WriteRegister(0x0B,*(input+12));
	DS1337_WriteRegister(0x0C,*(input+13));
	DS1337_WriteRegister(0x0D,*(input+14));
	DS1337_WriteRegister(0x0E,*(input+15));
	DS1337_WriteRegister(0x0F,*(input+16));
	
	outputData[0] = Header;
	outputData[1] = 0x05;
	outputData[2] = 0x00;
	outputData[3] = 0x1F;
	outputData[4] = 0x01;
	outputData[5] = ControlSumm(&outputData,5);
	
	for (unsigned char i=0;i<6;i++) USART_Transmit(outputData[i]);
}

void packet0x20(void) //Чтение состояния прерываний
{
		outputData[0] = Header;
		outputData[1] = 0x06;
		outputData[2] = 0x00;
		outputData[3] = 0x20;
		outputData[4] = (PIND & 0x04)>>2;
		outputData[5] = (PINB & 0x01);
		outputData[6] = ControlSumm(&outputData,6);
		
		for (unsigned char i=0;i<7;i++) USART_Transmit(outputData[i]);
}

void packet0x21(void) //Очистка eeprom
{
	bool Output = true;
	for (uint16_t i=0;i<EepromSize;i++) EEPROM_write (i,0xff);
	
	outputData[0] = Header;
	outputData[1] = 0x05;
	outputData[2] = 0x00;
	outputData[3] = 0x21;
	outputData[4] = Output;
	outputData[5] = ControlSumm(&outputData,5);
			
	for (unsigned char i=0;i<6;i++) USART_Transmit(outputData[i]);
	
	readStructDeviceInfo();
	readPoints();
}

void packet0x22(void) //Прочитать режим работы передатчика
{
	outputData[0] = Header;
	outputData[1] = 0x05;
	outputData[2] = 0x00;
	outputData[3] = 0x22;
	outputData[4] = currentTransmitMode;
	outputData[5] = ControlSumm(&outputData,5);
			
	for (unsigned char i=0;i<6;i++) USART_Transmit(outputData[i]);
}

void packet0x23(unsigned char *input) //Установить режим работы передатчика
{
	
	currentTransmitMode = *(input+1);
	
	writeStructDeviceInfo();
	PWM_SetMode(currentTransmitMode,countSignalPause,countSignal);
	
	outputData[0] = Header;
	outputData[1] = 0x05;
	outputData[2] = 0x00;
	outputData[3] = 0x23;
	outputData[4] = 0x01;
	outputData[5] = ControlSumm(&outputData,5);
	
	for (unsigned char i=0;i<6;i++) USART_Transmit(outputData[i]);
}

void packet0x24(void) //Прочитать параметры передатчика
{
	outputData[0] = Header;
	outputData[1] = 0x06;
	outputData[2] = 0x00;
	outputData[3] = 0x24;
	outputData[4] = countSignal;
	outputData[5] = countSignalPause;
	outputData[6] = ControlSumm(&outputData,6);
			
	for (unsigned char i=0;i<7;i++) USART_Transmit(outputData[i]);
}

void packet0x25(unsigned char *input) //Установить параметры передатчика
{
	
	countSignal = *(input+1);
	countSignalPause = *(input+2);
	
	writeStructDeviceInfo();
	PWM_SetMode(currentTransmitMode,countSignalPause,countSignal);
	
	outputData[0] = Header;
	outputData[1] = 0x05;
	outputData[2] = 0x00;
	outputData[3] = 0x25;
	outputData[4] = 0x01;
	outputData[5] = ControlSumm(&outputData,5);
	
	for (unsigned char i=0;i<6;i++) USART_Transmit(outputData[i]);
}

void packet0x26(void) //Прочитать параметры М-последовательсти
{
	outputData[0] = Header;
	outputData[1] = 0x0A;
	outputData[2] = 0x00;
	outputData[3] = 0x26;
	outputData[4] = *((unsigned char *)(&currentMSequence));
	outputData[5] = *(((unsigned char *)(&currentMSequence))+1);
	outputData[6] = *(((unsigned char *)(&currentMSequence))+2);
	outputData[7] = *(((unsigned char *)(&currentMSequence))+3);
	outputData[8] = currentMSequenceLen;
	outputData[9] = countSignalPause;
	outputData[10] = ControlSumm(&outputData,10);
	
	for (unsigned char i=0;i<11;i++) USART_Transmit(outputData[i]);
}

void packet0x27(unsigned char *input) //Установить параметры М-последовательсти
{
	bool result=true;

	*((unsigned char *)(&currentMSequence)) = *(input+1);;
	*(((unsigned char *)(&currentMSequence))+1) = *(input+2);
	*(((unsigned char *)(&currentMSequence))+2) = *(input+3);
	*(((unsigned char *)(&currentMSequence))+3) = *(input+4);
	
	currentMSequenceLen =  *(input+5);
	
	
	PWM_SetMSequence(currentMSequence,currentMSequenceLen);
	writeStructDeviceInfo();
	
	outputData[0] = Header;
	outputData[1] = 0x05;
	outputData[2] = 0x00;
	outputData[3] = 0x27;
	outputData[4] = result;
	outputData[5] = ControlSumm(&outputData,5);
	
	for (unsigned char i=0;i<6;i++) USART_Transmit(outputData[i]);
}
