

#define ADC_REF_INT ( (1 << REFS1) | (1 << REFS0) )
#define ADC_REF_AVCC ( (1 << REFS0) )
#define ADC_REF_AREF 0

void adc_init(uint8_t bitmask);
uint16_t read_adc(uint8_t chan);
