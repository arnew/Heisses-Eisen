#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


#include "adc.h"
#include "uart.h"
#include "display.h"
#include "encoder.h"
 

int main(void) {
	uart_init(UBRR_VAL);


//	TCCR0 |= (1 << CS02) | (1 << CS00); // Timer0, Clock/1024
//	TIMSK |= (1<<TOIE0); //Interrupt auf Overflow
	sei();
	while(1) {
		if (uart_str_complete == 1) {
			uart_puts("got: ");
			uart_puts(uart_buffer);
			uart_putc('\n');
			uart_str_complete = 0;
		}
		_delay_ms(500);
			uart_puts("Hallo Welt!\r\n");
		_delay_ms(500);
	}
	return 0; // (hopefully) never reached
}
