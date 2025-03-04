CC=avr-gcc
CFLAGS=-O1 -mmcu=atmega328p -I./ -Wall -Werror
OBJCOPY=avr-objcopy
OCFLAGS=-O ihex

MAIN_C=main.c
MAIN_OUT=./debug/main.out
MAIN_HEX=./debug/main.hex

VGA_H=vga.h
VGA_C=vga.c

$(MAIN_HEX): $(MAIN_OUT)
	$(OBJCOPY) $(OCFLAGS) $(MAIN_OUT) $(MAIN_HEX)

$(MAIN_OUT): $(MAIN_C) $(VGA_C) $(VGA_H)
	$(CC) -o $(MAIN_OUT) $(CFLAGS) $(MAIN_C) $(VGA_C)
