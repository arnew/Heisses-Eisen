PROJECT = heisseseisen
CC = avr-gcc
MCU = atmega8
TARGET = $(PROJECT).elf

F_CPU = 16000000

OBJECTS = uart.c adc.c display.c main.c 
COMMON = -mmcu=$(MCU)

CFLAGS = $(COMMON)
#CFLAGS += -Wall -Os -Wl,-u,vfprintf -lprintf_flt -lm
CFLAGS += -Wall -Os

CDEFS = -DF_CPU=$(F_CPU)UL
CFLAGS += $(CDEFS)

LDFLAGS = $(COMMON)

HEXFLAGS = -j .text -j .data

all: $(TARGET) $(PROJECT).hex

$(TARGET): $(OBJECTS)
	$(CC) $(CFLAGS) $(OBJECTS) -o $(TARGET)

%.hex: $(TARGET)
	avr-objcopy -O ihex $(HEXFLAGS) $< $@

clean: 
	-rm -rf $(PROJECT).elf $(PROJECT).hex

upload:
	-echo "fixme"
#	-avrdude -c avr910 -P /dev/ttyUSB0 -p m8 -i5 -U flash:w:$(PROJECT).hex -V
