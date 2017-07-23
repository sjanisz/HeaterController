#include <avr/io.h> 
#include <util/delay.h>
#include "lcd44780.h"
#define CHANNEL 0
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

//ENUMS
typedef enum {FIRST, SECOND, THIRD, FOURTH, FIRSTROW, SECONDROW} LCDPartEnum;					//enum for passing switch variables in SwitchLCDLine function
typedef enum {DEFAULT, FULLSPEED, SETTIME, CANCEL, TEMPERATURE, TIME} MenuStatesEnum;//enum for menu states
typedef enum {OFF, ONFULL, ONTIME, PC} ProgramStatesEnum;										//enum for program states 
//FUNCTIONS DECLARATIONS
void ZeroDetectionInterruptEnable();															//Enable interrupt from cross-zero circuit detector (ISR(INT2_vect))
void USARTCommunicationInitialization();														//Enable USART communication: transmitter and receiver enabled, interrupt from RX incoming data enabled, baud:9600, no parity, 1 stop bit
char USARTReceiveCharacter();																	//Receive 8-bit character from USART
void USARTSendCharacter(char characterToSend);													//Send 8-bit character from USART using UDR (USART Data Register)
void SPITemperatureCommunicationInitialization();												//Microcontroller role is a Master, Reading data on falling edge, saving on rising edge, sampling speed: fosc/64
uint16_t ReadFromSPI();																			//
void CheckForButtonPress();																		//Checking if any of four buttons was pressed
void SendTemperatureToComputerAndLCD();															//Sends temperature to LCD and through serial port to PC
void Timer0Initialization();																	//Timer0 initialization: fosc/1024, then overflow from timer occurs in following frequency:fosc/1024/255 (255 because it is overflow timer setup), what gives about 56Hz
void PIDRegulation(float setpointTemperature, float measuredTemperature);						//PID Regulator with setpoint temperature as input parameter
void SwitchLCDLine(LCDPartEnum lcdPart);														//Choose one of four segments where you want to display your up to 8 characters data
void USARTSendTemperature();																	//Send temperature to serial port

//****TODO
void TempInTime(uint16_t time, uint8_t setpointTemperature);// do zrobienia metodka
void TempMaxSpeed(uint8_t setpointTemperature);//max narost
float GetTemperature();

//GLOBAL VARIABLES
uint8_t HeaterPower=0;								//Variable that stores actual value of power (values from 1 to 100)
uint8_t MainSetpointTemperature = 50;				//This temperature is the temperature that user inputs through buttons or serial port
ProgramStatesEnum ProgramStates = OFF;				//States for controller work: OFF, ONFULL, ONTIME, PC
MenuStatesEnum MenuStates = DEFAULT;				//For switch in main function, shows switch what has to be printed on display
MenuStatesEnum PreviousMenuStates = TIME;			//There could be also any other case than TIME, it is set to TIME because otherwise it would be set to 0, and we would not be able to see all DEFAULT parameters right after program was run by user
uint16_t HeatingTime = 900;
uint16_t GlobalSecondTimer = 0;

//USART VARIABLES
//FlagPCControl=0
//FlagRegulationReset=0
//FlagTempMaxSpeed=0
//FlagTempInTime=0
//MainSetpointTemperature=50;
//heatingTimeChangeInMinutes=15;

//Two variables for changing temperature and heating time
uint8_t temperatureChange = 50;
uint8_t heatingTimeChangeInMinutes=15;

//FLAGS
uint8_t FlagRegulationINTOccur=0;					//Flag used to indicate whether any kind (full speed or in time) regulation should be executed in current loop (1) or not (0), data update interval: 1sec

uint8_t FlagRegulationReset = 0;					//Flag thah is being used to reset and start new full speed/temp in time heating sequence (0 means reset, 1 means that process is in process)
//uint8_t FlagTempMaxSpeedReset = 0; 				//Flag thah is being used to reset and start new full speed heating sequence (0 means reset, 1 means that process is in process)
//uint8_t FlagTempInTimeReset = 0;					//Flag that is being used to reset and start new temp in time sequence (0 means reset, 1 means that process is in process)

uint8_t FlagTempMaxSpeedProcess = 0;				//Flag used in main loop to indicate if max speed regulation is in process
uint8_t FlagTempInTimeProcess = 0;					//Flag used in main loop to indicate if temp in time regulation is in process

uint8_t FlagPCControl=0;							//0-controlling by switches, 1-controlling by PC
//MENU FLAGS
uint8_t FlagMenuSetOrView = 0;						//0 means viewing menu, 1 means setting variable

float PIDParameters[4];								//Holds all PID parameters (Kp, Ti, Td, Tp)
#define _KP 0
#define _TI 1
#define _TD 2
#define _TP 3

#define TIME_INTERVAL 56 							//preprocessor directive that is being used in timer0 overflow function to compare overflow counter to get 1s interval, TIME_INTERVAL = ( desired interval [sec] ) * ( frequency [Hz] ) / 255


//****TODO
//uint16_t on_off_timer = 270;//odczekanie po wyłączeniu regulacji dwustanowej = 240s
uint8_t sterowanie_on_off = 0;//wl/wyl sterowanie, nieużywane						

int main(void)
{
	sei();
	lcd_init();
	USARTCommunicationInitialization();
	SPITemperatureCommunicationInitialization();
	ZeroDetectionInterruptEnable();
	Timer0Initialization();
	
	lcd_cls();
	SwitchLCDLine(FIRST);
	lcd_str("IDLE");
	
	//Theoretical PID parameters: Kp=17.6, Ti=86.4, Td=14.4, Tp=1.0, other option for 50% step response: Kp=12.87, Ti=120, Td=20
	PIDParameters[_KP]=17.6;
	PIDParameters[_TI]=86.4;
	PIDParameters[_TD]=14.4;
	PIDParameters[_TP]=1.0;

	//There an uint16_t temp will be stored
	uint8_t temperatureToLCD;
	
	
	//Strings for program states
	const char *PROGRAMSTATESSTRINGS[4] =
	{
		"OFF","ONFULL","ONTIME","PC"
	};
	//initialize pins for buttons
	DDRD&=~(1<<PD2)|(1<<PD3)|(1<<PD5)|(1<<PD6);
	PORTD|=(1<<PD2)|(1<<PD3)|(1<<PD5)|(1<<PD6);
	
	//Initialize pin for triac output
	DDRD|=(1<<PD4);
	PORTD|=(1<<PD4);
	
	while(1)
	{	
		if(FlagPCControl==0)								//Control with buttons
		{
		CheckForButtonPress();
		}
		else												//Control with PC
		{	
			if(!(PIND&(1<<PD6)))							//ESCAPE
			{
				_delay_ms(80);
				if(!(PIND&(1<<PD6)))
				{
					FlagTempInTimeProcess=0;
					FlagTempMaxSpeedProcess=0;
					FlagRegulationReset=0;
					HeaterPower=0;
					ProgramStates=OFF;
					MenuStates=DEFAULT;
					FlagPCControl=0;
					_delay_ms(200);
				}
			}
			
			if(!(GlobalSecondTimer%60))USARTSendTemperature();
		
		}
		
		
		
		if(FlagRegulationINTOccur&&FlagTempMaxSpeedProcess)
		{
			TempMaxSpeed(MainSetpointTemperature);
			FlagRegulationINTOccur=0;
		}
		
		if(FlagRegulationINTOccur&&FlagTempInTimeProcess)
		{
			TempInTime(HeatingTime, MainSetpointTemperature);
			FlagRegulationINTOccur=0;
		}
		
		switch(MenuStates)//Switch for printing informations on LCD and set flags for buttons
		{
			//CASE DEFAULT
			case DEFAULT:
			
			if(PreviousMenuStates==DEFAULT)	
			{				
				if(GlobalSecondTimer%112)break;
			}
			
			temperatureToLCD = (int)GetTemperature();
			
			SwitchLCDLine(FIRST);
			lcd_str("PV:");
			if(temperatureToLCD!=(-1))
			{
				lcd_int(temperatureToLCD);
				lcd_str("C");
			}else
			{
				lcd_str("BT");
			}
			
			
				
			SwitchLCDLine(SECOND);
			lcd_str("S:");
			lcd_str(PROGRAMSTATESSTRINGS[ProgramStates]);
			
			SwitchLCDLine(THIRD);
			lcd_str("SP:");
			lcd_int((int)MainSetpointTemperature);
			lcd_str("C");
			
			SwitchLCDLine(FOURTH);												//change it, if time was set it will appear, otherwise empty place
			lcd_str("T:");
			lcd_int(HeatingTime/60);
			lcd_str("m");
			
			PreviousMenuStates = DEFAULT;
			break;
			//CASE FULL SPEED
			case FULLSPEED:
			if(PreviousMenuStates==FULLSPEED)break;
			SwitchLCDLine(FIRSTROW);
			lcd_str("    FUNCTION    ");
			SwitchLCDLine(SECONDROW);
			lcd_str("   FULL SPEED   ");
			PreviousMenuStates=FULLSPEED;
			break;
			//case SETTIME
			case SETTIME:
			if(PreviousMenuStates==SETTIME)break;
			SwitchLCDLine(FIRSTROW);
			lcd_str("    FUNCTION    ");
			SwitchLCDLine(SECONDROW);
			lcd_str("    SET TIME    ");
			PreviousMenuStates=SETTIME;
			break;
			//case PCCONTROL
			//case PCCONTROL:
			//if(PreviousMenuStates==PCCONTROL)break;
			//SwitchLCDLine(FIRSTROW);
			//lcd_str("    FUNCTION    ");
			//SwitchLCDLine(SECONDROW);
			//lcd_str("   PC CONTROL   ");
			//PreviousMenuStates=PCCONTROL;
			break;
			//case CANCEL
			case CANCEL:
			if(PreviousMenuStates==CANCEL)break;
			SwitchLCDLine(FIRSTROW);
			lcd_str("    FUNCTION    ");
			SwitchLCDLine(SECONDROW);
			lcd_str("     CANCEL     ");
			PreviousMenuStates=CANCEL;
			break;
			//case TEMPERATURE
			case TEMPERATURE:
			if((PreviousMenuStates==TEMPERATURE)&&(FlagMenuSetOrView==0))break;
			SwitchLCDLine(FIRSTROW);
			lcd_str("TEMPERATURE");
			SwitchLCDLine(SECONDROW);
			lcd_int(temperatureChange);
			lcd_str("C");
			if(FlagMenuSetOrView==1)
			{
				lcd_str("_");
				PreviousMenuStates=DEFAULT;
			}else
			{
				PreviousMenuStates=TEMPERATURE;
			}			
			break;
			//case TIME
			case TIME:
			if((PreviousMenuStates==TIME)&&(FlagMenuSetOrView==0))break;
			SwitchLCDLine(FIRSTROW);
			lcd_str("TIME");
			SwitchLCDLine(SECONDROW);
			lcd_int(heatingTimeChangeInMinutes);
			lcd_str("m");
			if(FlagMenuSetOrView==1)
			{
				lcd_str("_");
				PreviousMenuStates=DEFAULT;
			}else
			{
				PreviousMenuStates=TIME;
			}		
			break;
			//endcase
			default:
			break;
		}
		
		
		
	}



}


ISR(USART_RXC_vect)									//USART Interrupt for receiving data (needs to be modified)
{
	static char previousData = 'z';
	unsigned char data;
	data=UDR;
	
	//receieve byte data here (set temp and time)
	if(previousData=='t')
	{
		previousData='z';
		
		FlagTempInTimeProcess=0;
		FlagTempMaxSpeedProcess=0;
		FlagRegulationReset=0;
		HeaterPower=0;
		MenuStates=DEFAULT;
		MainSetpointTemperature=(uint8_t)data;
	}else if(previousData=='m')
	{
		previousData='z';
		
		FlagTempInTimeProcess=0;
		FlagTempMaxSpeedProcess=0;
		FlagRegulationReset=0;
		HeaterPower=0;
		MenuStates=DEFAULT;
		heatingTimeChangeInMinutes=(uint8_t)data;
		HeatingTime = 60*heatingTimeChangeInMinutes;
	}
	
	
	if(data=='o')
	{
		FlagTempInTimeProcess=0;
		FlagTempMaxSpeedProcess=0;
		FlagRegulationReset=0;
		HeaterPower=0;
		ProgramStates=PC;
		MenuStates=DEFAULT;
		FlagPCControl=1;
		//SEND DATA TO PC HERE (PARAMS)
		USARTSendByte(MainSetpointTemperature);
		USARTSendCharacter('t');
		USARTSendByte(heatingTimeChangeInMinutes);
		USARTSendCharacter('m');
	}else if (data=='c')
	{
		FlagTempInTimeProcess=0;
		FlagTempMaxSpeedProcess=0;
		FlagRegulationReset=0;
		HeaterPower=0;
		ProgramStates=OFF;
		MenuStates=DEFAULT;
		FlagPCControl=0;
	}else if (data=='t')
	{
		previousData = data;
	}else if (data=='m')
	{
		previousData = data;
	}else if (data=='s')
	{
		FlagTempInTimeProcess=0;		//CANCEL
		FlagTempMaxSpeedProcess=0;
		FlagRegulationReset=0;
		HeaterPower=0;
		MenuStates=DEFAULT;
	}else if (data=='f')
	{
		FlagTempInTimeProcess=0;
		FlagTempMaxSpeedProcess=1;
		FlagRegulationReset=0;
		HeaterPower=0;
		MenuStates=DEFAULT;
	}else if (data=='i')
	{
		FlagTempInTimeProcess=1;
		FlagTempMaxSpeedProcess=0;
		FlagRegulationReset=0;
		HeaterPower=0;
		MenuStates=DEFAULT;
	}
	
	//char *nalcd;

	//	USARTSendCharacter('-');
	//	USARTSendCharacter(data);
	//	USARTSendCharacter('-');
	//	lcd_cls();
	//	*nalcd=data;
	//	lcd_str(nalcd);
	//	USARTSendCharacter('\n');
	//	USARTSendCharacter('\r');
}


ISR(INT2_vect)										//Interrupt that occurs when MAINS voltage is passing zero (circuit connected to PB2)
{
	static uint8_t secondsCounter=0;		//This variable is being incremented every 100 zero crossings
	static uint8_t zeroCrossCounter=0;		//This variable is being incremented every single zero cross
	
	if(zeroCrossCounter==100)				//This if is being used to compare actual number of zero crosses to 100%
	{
		zeroCrossCounter=0;
		secondsCounter++;
		//SwitchLCDLine(FOURTH);
		//lcd_int((int)secondsCounter);
	}
	
	if(zeroCrossCounter<HeaterPower)		//HeaterPower is the count of sine halves that are supposed to power up heater, PD4 is triac output
	{
		PORTD&=~(1<<PD4);
	}
	else 
	{
		PORTD|=(1<<PD4);
	}
	
	zeroCrossCounter++;
}



ISR(TIMER0_OVF_vect)								//Timer0 overflow interrupt, occurs with 56Hz frequency, being used to set FlagRegulationINTOccur
{
	GlobalSecondTimer++;							//this must be here to prevent too frequently DEFAULT screen refreshing 
	
	static uint8_t timerCounter = 0;
	if(timerCounter<TIME_INTERVAL)
	{
		timerCounter++;
	}else											//There you can use variable that you want to access every second
	{
		FlagRegulationINTOccur=1;
		timerCounter=0;
	}
}



void SPITemperatureCommunicationInitialization()	//Initialize SPI, Master, CPHA- Read data on falling edge, write on rising, SPR1 means that sample speed is equal to fosc/64
{
	DDRB|=(1<<PB4)|(1<<PB7);
	SPCR|=(1<<SPE)|(1<<MSTR)|(1<<SPR1)|(1<<CPHA);
}


uint16_t ReadFromSPI()								//Read data from SPI
{
	uint16_t readFromSPIResult=0;	//1
	PORTB|=(1<<PB4);				//2
	_delay_ms(250);					//3
	PORTB&=~(1<<PB4);				//4 Four lines of code that are setting SS pin in LOW state
	_delay_ms(1);
	
	SPDR=0xFF;						//Sending data to SPDR register to receive data from temperature sensor
	while(!(SPSR&(1<<SPIF)));		//Waiting for data to be fetched
	readFromSPIResult=(SPDR<<8);	//Writing older part of variable to new result variable
	
	SPDR=0xFF;						//Another byte push to receive younger result byte
	while(!(SPSR&(1<<SPIF)));		//Waiting for data to be fetched
	readFromSPIResult+=SPDR;		//Writing younger part of variable to result variable
	
	return readFromSPIResult;		//return result
	
	
}


void USARTCommunicationInitialization() 			//Initialize function to communicate with USART
{
	UCSRB|=((1<<RXEN)|(1<<TXEN)|(1<<RXCIE)); 		//Unlock transmitter and receiver, enable receive data interrupt
	UCSRC|=((1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0)); 		//Need to set URSEL to access UCSRC, data bits number is 8bit
	UBRRL=95;
	UBRRH=(95>>8); 									//Setting baud rate to 9600bps
	
}

/*
char USARTReceiveCharacter() //Odbierz 8bitową daną 
{
	while(!(UCSRA&(1<<RXC)));
	return UDR;
}
*/
void USARTSendByte(uint8_t byteToSend)
{
	while(!(UCSRA&(1<<UDRE))); 						//Waiting for UDREmpty to be 0, if it is one we wait for registers to be emptied		
	UDR=byteToSend;
}
void USARTSendCharacter(char characterToSend) 		//Sending character using USART
{
	while(!(UCSRA&(1<<UDRE))); 						//Waiting for UDREmpty to be 0, if it is one we wait for registers to be emptied		
	UDR=characterToSend;
}



void ZeroDetectionInterruptEnable()					//Enable interrupt from cross-zero circuit detector (ISR(INT2_vect))
{
	DDRB&=~(1<<PB2);					//PB2 is input now
	MCUCSR&=~(1<<ISC2);					//Interrupt is being activated by falling edge
	GICR|=(1<<INT2);					//Enable interrupt from falling edge
}


void CheckForButtonPress()							//Buttons function
{
	/*
	if((!(PIND&(1<<PD2)))|(!(PIND&(1<<PD3))))
		{
			_delay_ms(30);
			if((!(PIND&(1<<PD2)))|(!(PIND&(1<<PD3))))
			{
				if(HeaterPower<100)
				{
					HeaterPower+=10;
				}
				
				_delay_ms(30);
			}
		}
		if(!(PIND&(1<<PD5)))
		{
			_delay_ms(30);
			if(!(PIND&(1<<PD5)))
			{
				if(HeaterPower>0)
				{
					HeaterPower-=10;
				}
				_delay_ms(30);
			}
		}
		if(!(PIND&(1<<PD6)))
		{
			_delay_ms(30);
			if(!(PIND&(1<<PD6)))
			{
				if(sterowanie_on_off)
				{
					sterowanie_on_off=0;
					HeaterPower=0;
				}
				else sterowanie_on_off=1;
			}
		}*/
		//{DEFAULT, FULLSPEED, SETTIME, PCCONTROL, CANCEL, TEMPERATURE, TIME}
		//UP/DOWN COUNTER for REPEATING
		static uint8_t UpCounter=0;
		static uint8_t DownCounter=0;
		static uint8_t FlagResetValue=1; //0 reset value, 1 exit (0 is set to 1 every time something is changing)
		
		if(!(PIND&(1<<PD2)))							//ENTER
		{
			_delay_ms(80);
			if(!(PIND&(1<<PD2)))						
			{
				if(FlagMenuSetOrView==0)
				{
					if		(MenuStates==DEFAULT)		;
					else if (MenuStates==FULLSPEED) 	
					{
						FlagTempInTimeProcess=0;
						FlagTempMaxSpeedProcess=1;
						FlagRegulationReset=0;
						HeaterPower=0;
						ProgramStates=ONFULL;
						MenuStates=DEFAULT;
					}
					else if (MenuStates==SETTIME)
					{
						FlagTempInTimeProcess=1;
						FlagTempMaxSpeedProcess=0;
						FlagRegulationReset=0;
						HeaterPower=0;
						ProgramStates=ONTIME;
						
						MenuStates=DEFAULT;
					}
					//else if (MenuStates==PCCONTROL)		;//fill this
					else if (MenuStates==CANCEL)		
					{
						FlagTempInTimeProcess=0;							//CANCEL
						FlagTempMaxSpeedProcess=0;
						FlagRegulationReset=0;
						HeaterPower=0;
						ProgramStates=OFF;
						
						MenuStates=DEFAULT;
					}
					else if (MenuStates==TEMPERATURE)	FlagMenuSetOrView=1;//change to setting (blinking C mayby?)
					else if (MenuStates==TIME)			FlagMenuSetOrView=1;//change to setting (blinking m mayby?)
				}else
				{
					if		(MenuStates==DEFAULT)		;
					else if (MenuStates==FULLSPEED) 	;
					else if (MenuStates==SETTIME)		;
					//else if (MenuStates==PCCONTROL)		;
					else if (MenuStates==CANCEL)		;
					else if (MenuStates==TEMPERATURE)	
					{
						FlagTempInTimeProcess=0;							//CANCEL
						FlagTempMaxSpeedProcess=0;
						FlagRegulationReset=0;
						HeaterPower=0;
						ProgramStates=OFF;
						
						MainSetpointTemperature = temperatureChange;
						FlagMenuSetOrView = 0;
					}
					else if (MenuStates==TIME)
					{
						FlagTempInTimeProcess=0;							//CANCEL
						FlagTempMaxSpeedProcess=0;
						FlagRegulationReset=0;
						HeaterPower=0;
						ProgramStates=OFF;
						
						HeatingTime = 60*heatingTimeChangeInMinutes;
						FlagMenuSetOrView = 0;
					}
				}	
				_delay_ms(200);
			}
		}
		if(!(PIND&(1<<PD3)))							//UP
		{
			_delay_ms(80);
			if(!(PIND&(1<<PD3)))
			{
				if(FlagMenuSetOrView==0)
				{
					if		(MenuStates==DEFAULT)		;
					else if (MenuStates==FULLSPEED) 	MenuStates=DEFAULT;
					else if (MenuStates==SETTIME)		MenuStates=FULLSPEED;
					//else if (MenuStates==PCCONTROL)		MenuStates=SETTIME;
					else if (MenuStates==CANCEL)		MenuStates=SETTIME;
					else if (MenuStates==TEMPERATURE)	MenuStates=CANCEL;
					else if (MenuStates==TIME)			MenuStates=TEMPERATURE;
				}else
				{
					if		(MenuStates==DEFAULT)		;
					else if (MenuStates==FULLSPEED) 	;
					else if (MenuStates==SETTIME)		;
					//else if (MenuStates==PCCONTROL)		;
					else if (MenuStates==CANCEL)		;
					else if (MenuStates==TEMPERATURE)	
					{
						FlagResetValue=0;
						if(temperatureChange<150)temperatureChange++;
					}
					else if (MenuStates==TIME)			
					{
						FlagResetValue=0;
						if(heatingTimeChangeInMinutes<255)
						{
							DownCounter=0;
							if(UpCounter<10)
							{
								UpCounter++;
								heatingTimeChangeInMinutes++;
							}
							else
							{
								if(heatingTimeChangeInMinutes>245)heatingTimeChangeInMinutes=255;
								else heatingTimeChangeInMinutes+=10;
								
							}								
						}
					}
				}
				_delay_ms(200);
			}
		}
		if(!(PIND&(1<<PD5)))							//DOWN
		{
			_delay_ms(80);
			if(!(PIND&(1<<PD5)))
			{
				if(FlagMenuSetOrView==0)
				{
					if		(MenuStates==DEFAULT)		MenuStates=FULLSPEED;
					else if (MenuStates==FULLSPEED) 	MenuStates=SETTIME;
					else if (MenuStates==SETTIME)		MenuStates=CANCEL;
					//else if (MenuStates==PCCONTROL)		MenuStates=CANCEL;
					else if (MenuStates==CANCEL)		MenuStates=TEMPERATURE;
					else if (MenuStates==TEMPERATURE)	MenuStates=TIME;
					else if (MenuStates==TIME)			;
				}else
				{
					if		(MenuStates==DEFAULT)		;
					else if (MenuStates==FULLSPEED) 	;
					else if (MenuStates==SETTIME)		;
					//else if (MenuStates==PCCONTROL)		;
					else if (MenuStates==CANCEL)		;
					else if (MenuStates==TEMPERATURE)	
					{
						FlagResetValue=0;
						if(temperatureChange>40)temperatureChange--;
					}
					else if (MenuStates==TIME)			
					{
						FlagResetValue=0;
						if(heatingTimeChangeInMinutes>15)
						{
							UpCounter=0;
							if(DownCounter<10)
							{
								DownCounter++;
								heatingTimeChangeInMinutes--;
							}
							else{
								if(heatingTimeChangeInMinutes<25)heatingTimeChangeInMinutes=15;
								else heatingTimeChangeInMinutes-=10;	
							}	
						}
					}
				}
				_delay_ms(200);
			}
		}
		if(!(PIND&(1<<PD6)))							//ESCAPE
		{
			_delay_ms(80);
			if(!(PIND&(1<<PD6)))
			{
				if(FlagMenuSetOrView==0)
				{
					if		(MenuStates==DEFAULT)		;
					else if (MenuStates==FULLSPEED) 	;
					else if (MenuStates==SETTIME)		;
					//else if (MenuStates==PCCONTROL)		;
					else if (MenuStates==CANCEL)		;
					else if (MenuStates==TEMPERATURE)	;
					else if (MenuStates==TIME)			;
				}else
				{
					if		(MenuStates==DEFAULT)		;
					else if (MenuStates==FULLSPEED) 	;
					else if (MenuStates==SETTIME)		;
					//else if (MenuStates==PCCONTROL)		;
					else if (MenuStates==CANCEL)		;
					else if (MenuStates==TEMPERATURE)	
					{
						if(FlagResetValue==0)
						{
							temperatureChange = MainSetpointTemperature;
							FlagResetValue=1;
						}else
						{
							FlagMenuSetOrView=0;
						}	
					}
					else if (MenuStates==TIME)			
					{
						if(FlagResetValue==0)
						{
							heatingTimeChangeInMinutes= HeatingTime/60;
							FlagResetValue=1;
						}else
						{
							FlagMenuSetOrView=0;
						}
					}
				}
				
				_delay_ms(200);
			}
		}
		
	
}

float GetTemperature()
{
	uint16_t readFromSPITemperature = ReadFromSPI();
	if(readFromSPITemperature&(1<<2))return -1;														//Check thermcouple presence, if thermcouple is not present, return -1
	readFromSPITemperature=readFromSPITemperature>>3; 												//Remove bits that do not store any information about temperature
	return (0.25 * readFromSPITemperature);
}
void SendTemperatureToComputerAndLCD()				//Send temperature to LCD and serial port
{	
	uint16_t readFromSPITemperature = ReadFromSPI();
	
	SwitchLCDLine(THIRD);
	lcd_int((int)HeaterPower);
			
	SwitchLCDLine(FIRST);
		
	if(readFromSPITemperature&(1<<2))
	{
		lcd_str("BT");
	}else
	{
		readFromSPITemperature=readFromSPITemperature>>3;
		int totalTemperature=(int)readFromSPITemperature*0.25;
		//uint8_t fractionTemperature=(uint8_t)((readFromSPITemperature&3)*25); //use this in the future if u want to see temperature in 0,25 celsjus degrees resolution
		lcd_int(totalTemperature);
		//lcd_char(',');
		//lcd_int(fractionTemperature);
			
		char pom[4];															//Preparing to send some data through serial port
		itoa(totalTemperature,pom,10);
		for(int i =0;(pom[i]!=0);i++)
		{						
			USARTSendCharacter(pom[i]);
		}
		USARTSendCharacter('#');												//# - # means that previous data was a temperature
	}
}


void Timer0Initialization()							//Initialize Timer0
{
	TCCR0|=(1<<CS00)|(1<<CS02);	//fosc/1024
	TIMSK|=(1<<TOIE0);			//TimerOverflow Interrupt Enable
}

void PIDRegulation(float setpointTemperature, float measuredTemperature)		//PID algorithm 
{	
	float measuredTemperaturePercentage = measuredTemperature/150.0;  								//getting measured temp in percents according to maximum heater temperature which is 150 Celsjus degrees
	float setpointTemperaturePercentage = setpointTemperature/150.0;  								//getting setpoint temp in percents according to maximum heater temperature which is 150 Celsjus degrees
	float error = setpointTemperaturePercentage - measuredTemperaturePercentage; 					//current error
	
	float u; 																						//Percentage value of regulator output signal (0-1, whichc means 0-100%)
	static float ISum=0, previousError=0;															//previousError - as it is, ISum - variable that stores sum for I part of PID
	static uint8_t IOverflow=0; 																	//definition of overflow variable that indicates whether regulator output signal was too low or too high
	
	if(FlagRegulationReset==0)																		//reseting PID params to prepare it for new regulation
	{
		ISum=0;
		previousError=0;
		IOverflow=0;
		return;
	}
	
	u=PIDParameters[_KP]*error; 																	//Proportional part of PID
	if(!IOverflow)ISum+=error; 																		//If overflow in previous PID loop run didnt occur, current error (SP-PV) is added to ISum
	u+=PIDParameters[_KP] * (PIDParameters[_TP]/PIDParameters[_TI]) * ISum; 						//Integral part of PID
	u+=PIDParameters[_KP] * (PIDParameters[_TD]/PIDParameters[_TP]) * (error - previousError); 		//Derrivate part of PID
	previousError=error; 																			//previousError saved for next PID loop
	IOverflow = 0; 																					//output signal exceed flag set to 0
	if(u>1.0){u=1.0;IOverflow=1; } 																	//checking if output signal did not exceed out of range
	else if(u<0.0){u=0.0;IOverflow=1; }; 															//checking if output signal did not exceed out of range
	
	HeaterPower=100*u;
}
void SwitchLCDLine(LCDPartEnum lcdPart)					//Used to place LCD text in one of four segments
{
	switch(lcdPart)
	{
		case FIRST:
		lcd_locate(0,0);
		lcd_str("        ");
		lcd_locate(0,0);
		break;
		case SECOND:
		lcd_locate(0,8);
		lcd_str("        ");
		lcd_locate(0,8);
		break;
		case THIRD:
		lcd_locate(1,0);
		lcd_str("        ");
		lcd_locate(1,0);
		break;
		case FOURTH:
		lcd_locate(1,8);
		lcd_str("        ");
		lcd_locate(1,8);
		break;
		case FIRSTROW:
		lcd_locate(0,0);
		lcd_str("                ");
		lcd_locate(0,0);
		break;
		case SECONDROW:
		lcd_locate(1,0);
		lcd_str("                ");
		lcd_locate(1,0);
		break;
	}
}

void TempMaxSpeed(uint8_t setpointTemperature)
{	
	float measuredTemperature = GetTemperature();
	
	static uint16_t onOffTimer;
	static uint8_t startPIDDiff;
	//Choose pair of values from below if else table
	if(FlagRegulationReset==0)
	{
		PIDRegulation(0.0,0.0);																		//Reset PID
		if(setpointTemperature<50) 
		{
			onOffTimer=270;
			startPIDDiff = 13;//bylo 12 musi byc 12 lub 13, zeby w ogole pogrzal troche, musi zostać 13 mimo 7st. przeskoku (start o 22stC, a co gdy start od 25stC?)
		}
		else if (setpointTemperature<60)
		{
			onOffTimer=265;
			startPIDDiff = 17;//bylo 13,16, 19 za duzo, 15 5 przeskok
		}
		else if (setpointTemperature<70)
		{
			onOffTimer=260;
			startPIDDiff = 18;//bylo 15, przy 17 pid zwariowal, 17 przeskoczyl o 5 st, dam 18, OK
		}
		else if (setpointTemperature<80)
		{
			onOffTimer=255;
			startPIDDiff = 18;//daje 18 bo przeskok byl o kilka stopni, OK!
		}
		else if (setpointTemperature < 90)
		{
			onOffTimer=250;
			startPIDDiff = 17;
		}
		else if (setpointTemperature <100)
		{
			onOffTimer=245;
			startPIDDiff = 16;
		}
		else if (setpointTemperature < 110)
		{
			onOffTimer=240;
			startPIDDiff = 16;
		}
		else if (setpointTemperature < 120)
		{
			onOffTimer=230;
			startPIDDiff = 16;
		}
		else if (setpointTemperature < 130)
		{
			onOffTimer=210;
			startPIDDiff = 15;
		}
		else if (setpointTemperature < 140)
		{
			onOffTimer=190;
			startPIDDiff = 15; 
		}
		else
		{
			onOffTimer=170;
			startPIDDiff = 12;
		}
		
		FlagRegulationReset=1;
	}
	
	
	if((int)measuredTemperature>=(setpointTemperature-startPIDDiff))												//This if is true if difference (SP-PV) is higher than specific value, else heater works with 100% power (maximum heating speed)
	{																						
		if(onOffTimer)
		{
			HeaterPower=0;
			onOffTimer--;
			return;
		}
		PIDRegulation(setpointTemperature, measuredTemperature);
	}else
	{
		HeaterPower = 100;
	}
}
void TempInTime(uint16_t time, uint8_t setpointTemperature)
{
	static uint8_t fromTemperature;
	static uint8_t toTemperature;
	static uint8_t tempDifference;
	
	static uint16_t tempInTimeCounter;
	static uint8_t actualTimeSetTemp;
	static uint16_t modulo;
	
	float measuredTemperature = GetTemperature(); 
	
	if(FlagRegulationReset==0)																		//First read current temperature to fromTemperature variable, next 
	{
		PIDRegulation(0.0,0.0);																		//Reset pid
		fromTemperature = (uint8_t)measuredTemperature;
		toTemperature = setpointTemperature;
		tempDifference = setpointTemperature - fromTemperature;
		
		tempInTimeCounter = 0;
		actualTimeSetTemp = fromTemperature +1;
		modulo = time/tempDifference;
		
		FlagRegulationReset=1;
	}
	tempInTimeCounter++;
	
	if((!(tempInTimeCounter%modulo))&&(actualTimeSetTemp<setpointTemperature))
	{
		actualTimeSetTemp++;
	}
	
	//SwitchLCDLine(SECOND); //enable this to check current set temperature for temp in time control
	//lcd_int((int)actualTimeSetTemp);
	
	PIDRegulation(actualTimeSetTemp, measuredTemperature);
}

void USARTSendTemperature()
{
	int totalTemperature=(int)GetTemperature();	
	
	char pom[4];															//Preparing to send some data through serial port
	itoa(totalTemperature,pom,10);
	for(int i =0;(pom[i]!=0);i++)
	{						
		USARTSendCharacter(pom[i]);
	}
	USARTSendCharacter('#');												//# - # means that previous data was a temperature
}