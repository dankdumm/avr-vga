#define F_CPU 8000000
#include"vga.h"
#include<avr/io.h>
#include<stdint.h>
#include<stdlib.h>

uint8_t mode = SVGA_800X600_60HZ;
uint8_t* img;

int main() {
    //PORTB will be the output bus;
    DDRC = 0b00011111;
    img = malloc(600);
    for(int i=0; i<600; i++) {
        *(img+i) = 1;
        //if(i <= 300) *(img+i) = 1;
        //else *(img+i) = 0;
    }

    vga_init(&PORTC, 800, 600, VGA_HPOL_POSITIVE | VGA_VPOL_POSITIVE);

    while(1) {
        vga_update(img);
        //While loop, doing nothing
    }

    return 0;
}
