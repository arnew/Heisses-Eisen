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

upload: all
	#sudo avrdude -c ft232r -p m8 -P ft0 -U flash:w:$(PROJECT).hex -V
	avrdude -c avrispmkII -p m8 -P usb -U flash:w:$(PROJECT).hex 

#programmer
#  id    = "ft232r";
#  desc  = "FT232R Synchronous BitBang";
#  type  = ft245r;
##CTS SCK
##RTS MOSI
##RI MISO
##DTR RST
#  miso  = 7;  # RI
#  sck   = 3;  # CTS
#  mosi  = 2;  # RTS
#  reset = 4;  # DTR ok
#;

	#-echo "fixme"
# tortilla ftdi bit bang:
#	-avrdude -c avr910 -P /dev/ttyUSB0 -p m8 -i5 -U flash:w:$(PROJECT).hex -V
