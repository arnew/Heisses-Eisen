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
	int on_periods;
} controlstate = {0,0,0,0,0};

#define STELL_MAX INT16_MAX
#define STELL_MIN INT16_MIN

volatile struct {
	int pulse;
	int rawduty;
	uint8_t duty;
	uint8_t zc_counter;
	uint8_t inter_period_counter;
} stats;

#define ZC_PERIOD 10

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
	uint16_t duty;
} s_calib;

typedef struct {
	int ticks;
} s_update;

volatile s_control control ={7,1,9,4    ,0,1000,    111,200};
volatile s_calib calib ={59,125,322,17};
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

inline static void triac_off() {
	PORTD &= ~(1 << PD7);
}

inline static void triac_on() {
	PORTD |= (1 << PD7);
}

int sense_temp() {
	triac_off();
	uint32_t adc_0_vals[4];
	uint8_t adci;
	for (adci = 0; adci < 4; adci++) {
		adc_0_vals[adci] = read_adc(0);
	}
	uint32_t adc_0 = (adc_0_vals[0] + adc_0_vals[1] + adc_0_vals[2] + adc_0_vals[3])/4;
	return calib.offset + adc_0*calib.numerator / calib.denominator;
}

void calc_pid(int temp) {
	controlstate.ist = temp;

	// TEMP STOP
	if(controlstate.ist > control.tmax) {
		controlstate.stell = 0;
		return;	
	}

	int32_t error = control.soll - controlstate.ist;
	int32_t p = error * control.p; //overflow unlikely
	controlstate.i += error;
	if(controlstate.i > control.imax) controlstate.i = control.imax; //handles overflow
	else if(controlstate.i < control.imin) controlstate.i = control.imin; //handles underflow
	int32_t i;
	if (control.i > 0) {
		i = controlstate.i / control.i;
	} else {
		i = 0;
	}
	int32_t d;
	if (control.d > 0) {
		d = (controlstate.ist - controlstate.d) / control.d;
		
	} else {
		d = 0;
	}
	controlstate.d = controlstate.ist;

	controlstate.stell = (p + i - d) * control.scale;
//	if ((p+i-d) >= 0) {
//		if (controlstate.stell < 0) { //handle overflow
//			controlstate.stell = STELL_MAX;
//		}
//	} else {
//		if (controlstate.stell > 0) { //handle overflow
//			controlstate.stell = STELL_MIN;
//		}
//	}
}

ISR(INT0_vect) {
	//100 Hz interrupt on ZC
	stats.zc_counter++;
//	every ZC_PERIODth interrupt
	stats.inter_period_counter = stats.zc_counter % (ZC_PERIOD);
	if(stats.inter_period_counter == 0) { // new interval has begun
//	sleep?
	_delay_us(2); //fixme
	int temp = sense_temp();
	calc_pid(temp);
//	controlstate.stell // -32000 .. + 32000
	if (controlstate.stell > 0) {
		//	controlvalue -> on_periods = 0..ZC_PERIOD-1
		controlstate.on_periods = 1; //fixme
	} else {
		controlstate.on_periods = 0;
	}
	}

	if (stats.inter_period_counter < controlstate.on_periods) {
		triac_on();
	} else {
		triac_off();
	}

	// do we need a rate limit by comparing with a timer?
////	if(bit_is_set(PIND, PD2)) { // rising edge only
//		stats.pulse++;
//		controlstate.ist = temp;
//
//		// TEMP STOP
//		if(controlstate.ist > control.tmax) return;
//
//		// todo: replace this with phasenanschnittsteuerung
//		if(controlstate.stell <= 0 ) {} 
//		else if((uint32_t)rand() * controlstate.stell >> calib.duty) {
//			stats.rawduty++;
//			PORTD |= ( 1 << PD7);
//		}
//
//		int32_t error = control.soll - controlstate.ist;
//		int32_t p = error * control.p;
//		controlstate.i += error;
//		if(controlstate.i > control.imax) controlstate.i = control.imax;
//		else if(controlstate.i < control.imin) controlstate.i = control.imin;
//		if (control.i > 0) {
//			int32_t i = controlstate.i / control.i;
//		} else {
//			int32_t i = 0;
//		}
//		if (control.d > 0) {
//			int32_t d = (controlstate.ist - controlstate.d) / control.d;
//			
//		} else {
//			int32_t d = 0;
//		}
//		controlstate.d = controlstate.ist;
//
//		controlstate.stell = (p + i - d) * control.scale;
//
//	} else {
//		PORTD &= ~( 1 << PD7);
//	}
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
	snprintf(buf,80,"pulsec: %i ipc: %i on_periods: %i\r\n",stats.pulse, stats.inter_period_counter, controlstate.on_periods);
	uart_puts(buf);
	snprintf(buf,80,"calib: y= %i/%i * x + %i duty: %u\r\n",calib.numerator, calib.denominator, calib.offset, calib.duty);
	uart_puts(buf);
	snprintf(buf,80,"EEPROM: %u\r\n",EEPROM_DATA_SIZE);
	uart_puts(buf);

}

void showupdate() {
	char buf[80];
	snprintf(buf,80,"%4x|soll: %3u ist: %3i i: %i d: %3i stell: %4i On: %2d #: %2d\r\n",
			ticks,
			control.soll,
			controlstate.ist ,controlstate.i,controlstate.d,
			controlstate.stell,
			controlstate.on_periods, stats.pulse
			 );
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
		case 'd': control.scale = atoi(p+1); break;

		case 'T': control.soll = atoi(p+1); break;
		case 'X': control.tmax = atoi(p+1); break;

		case 'B': calib.offset = atoi(p+1); break;
		case 'M': calib.numerator = atoi(p+1); break;
		case 'N': calib.denominator = atoi(p+1); break;
		case 'W': calib.duty = atoi(p+1); break;
		case 'H':
		default:
			  uart_puts(
					  "l load, s store, r reset EEPROM\r\n"
					  "R <ticks>: update rate\r\n"
					  "S: show status \r\n"
					  "\r\n"
					  "P <>: control P gain\r\n"
					  "I <>: control I gain\r\n"
					  "i <>: control Imin\r\n"
					  "j <>: control Imax\r\n"
					  "D <>: control D gain\r\n"
					  "d <>: control scale divisor\r\n"
					  "\r\n"
					  "T <>: control temperature\r\n"
					  "X <>: shutoff temperature\r\n"
					  "\r\n"
					  "calib: y=M/N * x + B\r\n"
					  "W <>: duty correction\r\n"
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
			if(nextupdate + update.ticks <ticks) ticks = -1-ticks;
			nextupdate = ticks + update.ticks;

		}
	}
	return 0; // (hopefully) never reached
}
