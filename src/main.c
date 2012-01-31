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


void report_adc_val() {
	uint16_t res = read_adc(0);
	uint16_t t = res/2;
	char buf[20];
	//sprintf(buf, "ADC: %1.3f\r\n", 1.222);
	if (res % 2 == 1) {
		sprintf(buf, "read: %u, temp: %u.5 degC\r\n", res, t);
	} else {
		sprintf(buf, "read: %u, temp: %u.0 degC\r\n", res, t);
	}
	uart_puts(buf);
}

int main(void) {
	DDRC = 0x00;
	uart_init(UBRR_VAL);
	adc_init(ADC_REF_INT);

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
