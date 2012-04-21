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

volatile struct {
	int ticks;
	int next;
} update = {60,0};

volatile struct {
	int p,i,d;
	int scale;
	uint8_t imin,imax;
	uint16_t soll;
	uint16_t tmax;
} control={100,1,500,10,0,50,100,375};

volatile struct {
	uint8_t d;
	uint8_t i;
	uint16_t ist;
	int stell;
} controlstate = {0,0,0,0};

volatile struct {
	int pulse;
	int rawduty;
	uint8_t duty;
} stats;

volatile struct {
	int offset;
	int numerator;
	int denominator;
} calib = {50,2,5};

ISR(INT0_vect) {
	// do we need a rate limit by comparing with a timer?
	if(bit_is_set(PIND, PD2)) { // rising edge only
		stats.pulse++;
		controlstate.ist = calib.offset + (int)read_adc(0)*calib.numerator / calib.denominator;

		if(controlstate.ist > control.tmax) return;

		int error = control.soll - controlstate.ist;
		int p = control.p * error;
		controlstate.i += error;
		if(controlstate.i > control.imax*control.scale) controlstate.i = control.imax*control.scale;
		else if(controlstate.i < control.imin*control.scale) controlstate.i = control.imin*control.scale;
		int i = control.i * controlstate.i;
		int d = control.d * (controlstate.ist - controlstate.d) ;
		controlstate.d = controlstate.ist;

		controlstate.stell = (p + i - d) / control.scale;

		if(controlstate.stell < 0 ) return;

		// todo: replace this with phasenanschnittsteuerung
		if(controlstate.stell > 100 || rand() < controlstate.stell * RAND_MAX) {
			stats.rawduty++;
			PORTD |= ( 1 << PD7);
			_delay_us(500);
			PORTD &= ~( 1 << PD7);
		}
	}
}

int ticks = 0;
ISR(TIMER0_OVF_vect)
{
	ticks++;
}

void showstatus() {
	char buf[80];
	snprintf(buf,80,"PID: P: %i I: %i D: %i imin: %u imax: %u\r\n",control.p,control.i, control.d, control.imin, control.imax);
	uart_puts(buf);
	snprintf(buf,80,"calib: y= %i/%i * x + %i\r\n",calib.numerator, calib.denominator, calib.offset);
	uart_puts(buf);

}

void showupdate() {
	char buf[80];
	snprintf(buf,80,"soll: %u ist: %i i: %i d: %i stell: %i On: %d #: %d duty: %u%%\r\n",
			control.soll,
			controlstate.ist ,controlstate.i,controlstate.d,
			controlstate.stell,
			stats.rawduty, stats.pulse,
			100 * stats.rawduty  / stats.pulse );
	stats.pulse = stats.rawduty=0;
	uart_puts(buf);
}


void process_cmd(char *p) {
	switch(*p) {
		case 'S': showstatus(); break;
		case 'R': update.ticks = atoi(p+1); break;
		case 'P': control.p = atoi(p+1); break;
		case 'I': control.i = atoi(p+1); break;
		case 'i': control.imin = atoi(p+1); break;
		case 'j': control.imax = atoi(p+1); break;
		case 'D': control.d = atoi(p+1); break;
		case 'T': control.soll = atoi(p+1); break;
		case 'X': control.tmax = atoi(p+1); break;
		case 'B': calib.offset = atoi(p+1); break;
		case 'M': calib.numerator = atoi(p+1); break;
		case 'N': calib.denominator = atoi(p+1); break;
		case 'H':
		default:
			  uart_puts(
					  "R <ticks>: update rate\r\n"
					  "S: show status \r\n"
					  "\r\n"
					  "P <>: control P gain\r\n"
					  "I <>: control I gain\r\n"
					  "D <>: control D gain\r\n"
					  "\r\n"
					  "T <>: control temperature\r\n"
					  "X <>: shutoff temperature\r\n"
					  "i <>: control Imin\r\n"
					  "j <>: control Imax\r\n"
					  "\r\n"
					  "calib: y=M/N * x + B\r\n"
				   );
			  break;
	}
}

int main(void) {
	DDRC = 0x00;

	// enable interrupt
	GICR = (1 << INT0);
	MCUCR = (1 << ISC00);

	uart_init(UBRR_VAL);
	adc_init(ADC_REF_INT);

	// eingang + pullup
	DDRD &=  ~(1 << PD2);
	PORTD &= ~(1 << PD2);

	// D7 output, low
	DDRD |= (1 << PD7) | (1<<PD6);
	PORTD &= ~( 1 << PD7);
	PORTD &= ~( 1 << PD6);

	// about 61 ticks per second
	TCCR0 |= (1 << CS02) | (1 << CS00); // Timer0, Clock/1024
	TIMSK |= (1<<TOIE0); //Interrupt auf Overflow

	sei();
	_delay_ms(1000);
	while(1) {
		if (uart_str_complete == 1) {
			cli();
			uart_puts("got: ---");
			uart_puts(uart_buffer);
			uart_puts("---\r\n");
			process_cmd(uart_buffer);
			uart_str_complete = 0;
			sei();
		}
		if(update.next < ticks) {
			showupdate();
			update.next = ticks + update.ticks;
		}
	}
	return 0; // (hopefully) never reached
}
