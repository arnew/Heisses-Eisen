#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <avr/eeprom.h>
#include <util/crc16.h>

#include "adc.h"
#include "uart.h"
#include "display.h"
#include "encoder.h"

extern uint8_t uart_str_complete; 		// global status indicating pending rx'd line
extern volatile uint8_t uart_str_count; 		// points to next "free" array index
extern uchar_t uart_buffer[];

volatile struct {
	int d;
	int i;
	int ist;
	int stell;
} controlstate = {0,0,0,0};

volatile struct {
	int pulse;
	int rawduty;
	uint8_t duty;
} stats;

typedef struct {
	int p,i,d;
	int scale;
	int imin,imax;
	uint16_t soll;
	uint16_t tmax;
} s_control;

typedef struct {
	int offset;
	int numerator;
	int denominator;
} s_calib;

typedef struct {
	int ticks;
} s_update;

volatile s_control control ={7,1,9,4    ,0,1000,    111,200};
volatile s_calib calib ={59,125,322};
volatile s_update update = {60};

typedef struct {
	s_control ctl;
	s_calib cal;
	s_update upd;
	uint16_t crc;
} s_eeprom;

#define EEPROM_DATA_SIZE (sizeof(s_eeprom))

void eeprom_reset() {
	int i;
	for(i = 0; i<sizeof(s_eeprom) ; i++) {
		eeprom_busy_wait();
		eeprom_write_byte((void*)i,i);
	}
}

void eeprom_save() {
	cli();
	s_eeprom data = {control,calib,update,0};
	uint8_t *p = (uint8_t*)&data;
	int i;
	for(i = 0; i<sizeof(data) ; i++) {
		eeprom_busy_wait();
		eeprom_write_byte((void*)i,p[i]);
		if(i<sizeof(data)-2) data.crc = _crc16_update(data.crc,p[i]);
	}
	sei();
}

int eeprom_get() {
	cli();
	s_eeprom data = {{0}};
	uint16_t crc = 0;
	uint8_t *p = (uint8_t*)&data;
	int i;
	for(i = 0; i<sizeof(data) ; i++) {
		eeprom_busy_wait();
		p[i] = eeprom_read_byte((void*)i);
		crc = _crc16_update(crc,p[i]);
	}
	if(crc != 0) {
		sei();
		return 0;
	}
	control = data.ctl;
	calib = data.cal;
	update = data.upd;
	sei();
	return 1;
}


ISR(INT0_vect) {
	// do we need a rate limit by comparing with a timer?
	if(bit_is_set(PIND, PD2)) { // rising edge only
		stats.pulse++;
		int temp = calib.offset + ((uint32_t)read_adc(0))*calib.numerator / calib.denominator;
		controlstate.ist+=temp;
		controlstate.ist>>=1;

		if(controlstate.ist > control.tmax) return;

		int32_t error = control.soll - controlstate.ist;
		int32_t p = error << control.p;
		controlstate.i += error;
		if(controlstate.i > control.imax) controlstate.i = control.imax;
		else if(controlstate.i < control.imin) controlstate.i = control.imin;
		int32_t i = controlstate.i << control.i;
		int32_t d = (controlstate.ist - controlstate.d) << control.d;
		controlstate.d = controlstate.ist;

		controlstate.stell = (p + i - d) >> control.scale;

		if(controlstate.stell < 0 ) return;

		// todo: replace this with phasenanschnittsteuerung
		if((uint32_t)rand() * controlstate.stell/10 / RAND_MAX) {
			stats.rawduty++;
			PORTD |= ( 1 << PD7);
		}
	} else {
		PORTD &= ~( 1 << PD7);
	}
}

int ticks = 0;
int nextupdate;
ISR(TIMER0_OVF_vect)
{
	ticks++;
}

void showstatus() {
	char buf[80];
	snprintf(buf,80,"PID: P: %i I: %i D: %i imin: %i imax: %i\r\n",control.p,control.i, control.d, control.imin, control.imax);
	uart_puts(buf);
	snprintf(buf,80,"calib: y= %i/%i * x + %i\r\n",calib.numerator, calib.denominator, calib.offset);
	uart_puts(buf);
	snprintf(buf,80,"EEPROM: %u\r\n",EEPROM_DATA_SIZE);
	uart_puts(buf);

}

void showupdate() {
	char buf[80];
	snprintf(buf,80,"soll: %3u ist: %3i i: %i d: %3i stell: %4i On: %2d #: %2d duty: %3u%%\r\n",
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
		case 'l': if(eeprom_get()) { uart_puts("params read:\r\n"); showstatus(); } 
			  else { uart_puts("params read fail\r\n"); }; 
			  break;
		case 's': eeprom_save(); break;
		case 'r': eeprom_reset(); break;

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

	eeprom_get();
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
		if(nextupdate < ticks) {
			showupdate();
			nextupdate = ticks + update.ticks;
		}
	}
	return 0; // (hopefully) never reached
}
