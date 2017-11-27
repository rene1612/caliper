/*
 * Caliper.c
 *
 * Created: 09.11.2017 14:12:04
 * Author : Rene Schönrock
 */ 

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <string.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <avr/wdt.h>


#include "uart.h"


/* define CPU frequency in Hz in Makefile */
#ifndef F_CPU
#error "F_CPU undefined, please define CPU frequency in Hz in Makefile"
#endif

#ifndef CLIPER_COUNT
#define CLIPER_COUNT 1
#endif

//#define CALIPER_0_NENABLE (DDRC & (1<<PINC0))
#define CALIPER_0_DATA (PINB & (1<<PINB1))

#define TIMOUT_TIMER_START TCCR0B|=(_BV(CS01)|_BV(CS00))
#define TIMOUT_TIMER_STOP TCCR0B&=0xF8
#define TIMOUT_TIMER_SET(val) OCR0A=val; TCNT0 = 0
#define START_TIMEOUT (F_CPU/(64000000/40))
#define TRANSFER_TIMEOUT (F_CPU/(64000000/900))

/* Define UART buad rate here */
#define UART_BAUD_RATE      115200


enum TIMEOUT_MODE {
	WAIT_FOR_START,
	WAIT_FOT_TRANSFER_COMPLETED,
	};
	
enum TIMEOUT_MODE timeout_mode;
	
	
uint8_t current_caliper;

//struct caliper {
	//uint8_t update_flag;
	//uint8_t bit_count;
	//unsigned int sync_timeout;
	//signed long absolute;
	//signed long relatgive;
	//};

struct caliper {
	uint8_t update_flag;
	uint8_t bit_mask;
	//unsigned int sync_timeout;
	uint8_t buffer_index;
	uint8_t nr;
	uint8_t buffer[6];
	uint16_t crc;
};

struct caliper calipers[CLIPER_COUNT];


void activate_caliper(uint8_t nr)
{
	DDRC &= 0xF8;
	DDRC |= (1<<nr);
}


/************************************************************************
 * @fn		ISR(INT0_vect)
 * @brief	ISR für 
 *
 * Ausführliche Beschreibung
 *
 * @param   keine 
 * @return	keine
 */
//ISR(INT0_vect)
//{
	////int8_t bit_mask;
	//if (calipers[0].update_flag)
		//return;
//
	//
	//if (!(CALIPER_0_DATA))
		//calipers[0].buffer[calipers[0].buffer_index] |= calipers[0].bit_mask;
	//
		//
////	calipers[0].buffer[calipers[0].buffer_index] |= (bit_mask << calipers[0].bit_count);
	//
	//calipers[0].bit_mask<<=1;
	//
	//if (!calipers[0].bit_mask)
	//{
		//calipers[0].bit_mask=1;
		//if (++calipers[0].buffer_index>=6) {
			//calipers[0].update_flag = 1;
		//}
	//}
		//
//}

/************************************************************************
 * @fn		ISR(INT1_vect)
 * @brief	ISR für 
 *
 * Ausführliche Beschreibung
 *
 * @param   keine 
 * @return	keine
 */
ISR(INT0_vect)
{
	timeout_mode = WAIT_FOR_START;
	TIMOUT_TIMER_SET(START_TIMEOUT);
	TIMOUT_TIMER_START;
}

/************************************************************************
 * @fn		ISR(INT1_vect)
 * @brief	ISR für 
 *
 * Ausführliche Beschreibung
 *
 * @param   keine 
 * @return	keine
 */
ISR(TIMER0_OVF_vect)
{
	
		switch(timeout_mode){
			case WAIT_FOR_START:
				//wir haben wahrscheinlich einen gültigen Start getroffen
				//Datentransfer initialisieren
				EIMSK &= ~_BV(INT0);
				TIMOUT_TIMER_SET(TRANSFER_TIMEOUT);
				
				calipers[current_caliper].buffer_index = 0;
				//USISR = 0; //Zähler und Int-Flags zurücksetzen
				//USICR = (1<<USIOIE) | (1<<USIWM0) | (1<<USICS1);
				timeout_mode = WAIT_FOT_TRANSFER_COMPLETED;
				SPCR |= _BV(SPE);
				//TIMOUT_TIMER_STOP;
				break;
			
			case WAIT_FOT_TRANSFER_COMPLETED:
				//Timeout für Transfer ist aufgelaufen (Neu durchstarten)
				SPCR &= ~_BV(SPE);
				timeout_mode = WAIT_FOR_START;
				EIMSK = _BV(INT0);

				break;
		}
}

/************************************************************************
 * @fn		ISR(INT1_vect)
 * @brief	ISR für 
 *
 * Ausführliche Beschreibung
 *
 * @param   keine 
 * @return	keine
 */
ISR(SPI_STC_vect)
{
	calipers[current_caliper].buffer[calipers[current_caliper].buffer_index] = SPDR;
	
	if(calipers[current_caliper].buffer_index++ >= 6){
		
		TIMOUT_TIMER_STOP;
		calipers[current_caliper].update_flag = 1;
		
		if (current_caliper++ >= CLIPER_COUNT)
			current_caliper = 0;
			
		activate_caliper (current_caliper);
		timeout_mode = WAIT_FOR_START;
		EIMSK = _BV(INT0);
	}
}

/************************************************************************
 * @fn		void init_Sys(void)
 * @brief	Systeminitialisierung
 *
 * Ausführliche Beschreibung
 *
 * @param   keine 
 * @return	keine
 */
void init_Sys(void)
{
	uint8_t i;
	
	//DDRB &= ~(_BV(PB0) | _BV(PB1));
	//DDRD &= ~(_BV(PD2) | _BV(PD3));
	
	PORTC &= 0xF8;

	
	//MCUCR |= _BV(ISC00) | _BV(ISC01) | _BV(ISC10) | _BV(ISC11); //steigende Flanke von Int0 und Int2 erzeugen Interrupt
	//GIMSK |= _BV(INT0) | _BV(INT1);
	EICRA = _BV(ISC01);
	
	TCCR0A = _BV(WGM01) | _BV(WGM00);
	TCCR0B = _BV(WGM02);
	TIMSK0 = _BV(TOV0);
	
	SPCR =  _BV(SPIE) | _BV(DORD);

    /*
     *  Initialize UART library, pass baudrate and AVR cpu clock
     *  with the macro 
     *  UART_BAUD_SELECT() (normal speed mode )
     *  or 
     *  UART_BAUD_SELECT_DOUBLE_SPEED() ( double speed mode)
     */
    uart_init( UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU) ); 
	
	for (i=0;i<CLIPER_COUNT;i++) {
		calipers[i].update_flag=0;
		calipers[i].bit_mask=1;
		calipers[i].buffer_index=0;
		calipers[i].nr=i;
		memset((void*)calipers[i].buffer, 0, sizeof(calipers[i].buffer));
		calipers[i].crc = 0;
	}
	
	current_caliper = 0;
	activate_caliper(current_caliper);
	EIMSK = _BV(INT0);

	//Interrupts anwerfen
	sei();
	
#ifdef __USE_WDT__
	//Watchdogtimer aktivieren  	WDTO_250MS
	//wdt_enable(WDTO_60MS);
	wdt_enable(WDTO_250MS);
#else
	wdt_disable();
#endif
}

//uint8_t buffer[11];

//float rel;

int main(void)
{
    
    init_Sys();
	
    /*
     *  Transmit string to UART
     *  The string is buffered by the uart library in a circular buffer
     *  and one character at a time is transmitted to the UART using interrupts.
     *  uart_puts() blocks if it can not write the whole string to the circular 
     *  buffer
     */
    uart_puts("Start\r\n");
    
    /* 
     * Use standard avr-libc functions to convert numbers into string
     * before transmitting via UART
     */     
   // itoa( num, buffer, 10);   // convert interger into string (decimal format)         
    //uart_puts(buffer);        // and transmit string to UART

    
    /*
     * Transmit single character to UART
     */
    uart_putc('\r');
    /* Replace with your application code */
    while (1) 
    {
#ifdef __USE_WDT__
//Watchdog zurücksetzen
wdt_reset();
#endif

		for(uint8_t i=0; i<CLIPER_COUNT; i++) {
			
			if (calipers[i].update_flag) {
				
				//buffer[0]=i;
//				*(float*)(buffer+1)=(calipers[i].absolute/100.0);
//				*(float*)(buffer+5)=(calipers[i].relatgive/100.0);

				//rel = calipers[i].relatgive/100.0;
				//sprintf(buffer, "%3.2f\r", rel);
				//uart_puts(buffer);
				
				
				//*(signed long*)(buffer+1)=calipers[i].absolute;
				//*(signed long*)(buffer+5)=calipers[i].relatgive;
				//buffer[9]=0x00;
				//buffer[10]=0x00;
				
				uart_write((unsigned char*)&calipers[i].nr, 9);
				
				//calipers[i].bit_count=0;
				//calipers[i].absolute=0;
				//calipers[i].relatgive=0;
				calipers[i].bit_mask=1;
				calipers[i].buffer_index=0;
				memset((void*)calipers[i].buffer, 0, sizeof(calipers[i].buffer));

				calipers[i].update_flag = 0;

    //uart_puts(".");
			}
		}
		
		//uart_puts(".");

    }
}

