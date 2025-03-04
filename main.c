#define F_CPU 8000000
#include"vga.h"
#include<avr/io.h>
#include<stdint.h>

uint8_t mode = SVGA_800X600_60HZ;

int main() {
    //PORTB will be the output bus;
    DDRC = 0b00011111;
    vga_init(&PORTC, 800, 600, VGA_HPOL_POSITIVE | VGA_VPOL_POSITIVE);

    while(1) {
        //vga_update();
        //While loop, doing nothing
    }

    return 0;
}
