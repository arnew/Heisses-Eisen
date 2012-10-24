// uart.c
//
//

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include "uart.h"


uint8_t uart_str_complete=0; 		// global status indicating pending rx'd line
volatile uint8_t uart_str_count=0; 		// points to next "free" array index
uchar_t uart_buffer[UART_MAX_STR_LEN+1]="";


inline void uart_init(uint16_t ubrr)
{
	UBRRH = (uint8_t) (ubrr >> 8);
	UBRRL = (uint8_t) ubrr;

	UCSRB = (1<<RXEN) | (1<<RXCIE) | (1<<TXEN);  // UART TXRX einschalten, RX Interrupt
	UCSRC = (1<<URSEL) | (1<<UCSZ1) | (3<<UCSZ0);  // Asynchron 8N1
}

void uart_putc(uchar_t c)
{
    while (!(UCSRA & (1<<UDRE)))  /* warten bis Senden moeglich */
    {
    }                             
 
    UDR = c;   
}

void uart_puts (uchar_t *s)
{
    while (*s)
    {   
        uart_putc(*s);
        s++;
    }
}

uchar_t uart_getc(void) {
	while (!(UCSRA & (1 << RXC) )) {
	}
	return UDR;
}


ISR(USART_RXC_vect) {
	uchar_t buf = UDR; // only read once!
	if (uart_str_complete == 0) {
		if (buf !='\n' && buf != '\r' && uart_str_count < UART_MAX_STR_LEN) {
			if( buf == 0x1b && uart_str_count == 0 ) { // RESET ON ESC AS FIRST CHAR
				wdt_enable(WDTO_15MS); while(1);
			}

			uart_buffer[uart_str_count++] = buf;
		} else {
			uart_buffer[uart_str_count] = '\0';
			uart_str_count = 0;
			uart_str_complete = 1;
		}
	}
	return ;
}


