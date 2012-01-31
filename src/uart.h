#include "types.h"

#define BAUD 1000000UL// Baudrate !! UL !!

#define UART_MAX_STR_LEN 80
// uart_buffer holds UART_MAX_STR_LEN uchar_ts + final \0
 
extern uint8_t uart_str_complete; 		// global status indicating pending rx'd line
extern volatile uint8_t uart_str_count; 		// points to next "free" array index
extern uchar_t uart_buffer[];


// Berechnungen
#define UBRR_VAL ((F_CPU+BAUD*8)/(BAUD*16)-1)   // clever runden
#define BAUD_REAL (F_CPU/(16*(UBRR_VAL+1)))     // Reale Baudrate
#define BAUD_ERROR ((BAUD_REAL*1000)/BAUD) // Fehler in Promille, 1000 = kein Fehler.
//  
#if ((BAUD_ERROR<990) || (BAUD_ERROR>1010))
  #error Systematischer Fehler der Baudrate BAUD_ERROR grösser 1% und damit zu hoch! 
#endif 

//prototypes
void uart_init(uint16_t ubrr);
void uart_putc(uchar_t c);
void uart_puts(uchar_t *s);
uchar_t uart_getc(void);
