#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <util/delay.h>


#include "adc.h"
#include "uart.h"
#include "display.h"
#include "encoder.h"
 
extern uint8_t uart_str_complete; 		// global status indicating pending rx'd line
extern volatile uint8_t uart_str_count; 		// points to next "free" array index
extern uchar_t uart_buffer[];

int foo;

ISR(INT0_vect) {
	foo ++;
	if(foo%2) {
		PORTD |= ( 1 << PD7);
	} else {
		PORTD &= ~( 1 << PD7);
	}
}

void report_adc_val() {
	uint16_t res0 = read_adc(0);
	uint16_t res1 = read_adc(1);
	char buf[30];
	snprintf(buf,30,"%u,%u: %u.%c\r\n",res0,res1, res0>>1, (res0&1)?'5':'0');
	uart_puts(buf);
}

int main(void) {
	DDRC = 0x00;

	// enable interrupt
	GICR = (1 << INT0);
	MCUCR = (1 << ISC10);

	uart_init(UBRR_VAL);
	adc_init(ADC_REF_INT);

	// eingang + pullup
	DDRD &=  ~(1 << PD2);
	PORTD &= ~(1 << PD2);


	// D7 output, low
	DDRD |= (1 << PD7);
	PORTD &= ~( 1 << PD7);

//	TCCR0 |= (1 << CS02) | (1 << CS00); // Timer0, Clock/1024
//	TIMSK |= (1<<TOIE0); //Interrupt auf Overflow
	sei();
	_delay_ms(1000);
	while(1) {
		if (uart_str_complete == 1) {
			cli();
			uart_puts("got: ");
			uart_puts(uart_buffer);
			uart_puts("\r\n");
			uart_str_complete = 0;
			sei();
		}
		_delay_ms(200);
		report_adc_val();
			//uart_puts("Hallo Welt!\r\n");
	}
	return 0; // (hopefully) never reached
}
