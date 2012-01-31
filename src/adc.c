
#include <avr/io.h>


uint16_t read_adc(uint8_t chan) {
	ADMUX &= 240; //delete lowest 4 bit
	ADMUX |= chan; //select channel
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC)); //wait for completion
	return ADCW;
}


void adc_init(uint8_t bitmask) {
	ADCSRA = (1 << ADEN) | (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2);
	ADMUX = bitmask;
	read_adc(0); //first init
}
