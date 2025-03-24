#ifndef VGA_H
#define VGA_H
#include<stdint.h>

/*
All measurements of time are in units of us (microseconds)
//TODO: How should we round-off timing info that are non-whole numbers?
*/

/*--VGA SIGNAL DISPLAY MODES:--*/
#define SVGA_800X600_60HZ 0
//SVGA 800X600 @ 60HZ: HORIZONTAL TIMING INFO (us)
/*
DISPLAY - 20us
FPORCH - 1us
HSYNC - 3us
BPORCH - 2us
*/
#define SVGA_800X600_HLINE0 (20+1)
#define SVGA_800X600_HLINE (2+20+1)
#define SVGA_800X600_HYSNC (3)
#define SVGA_HALL_TEST (2+20+1)
/*
#define SVGA_800X600_60HZ_HDIS 20 
#define SVGA_800X600_60HZ_HFPORCH (1+20)
#define SVGA_800X600_60HZ_HSYNC (3+1+20)
#define SVGA_800X600_60HZ_HBPORCH (2+3+1+20)
*/
//SVGA 800X600 @ 60HZ: VERTICAL TIMING INFO (lines)
/*
DISPLAY - 600 lines
FPORCH - 1 line
VSYNC - 4 lines
VPORCH - 23 lines
*/
#define SVGA_800X600_60HZ_VDIS 600
#define SVGA_800X600_60HZ_VFPORCH 1+600
#define SVGA_800X600_60HZ_VSYNC 4+1+600
#define SVGA_800X600_60HZ_VBPORCH 23+4+1+600

//VGA SIGNAL INFO:
#define VGA_HPOL_NEGATIVE 0b0
#define VGA_HPOL_POSITIVE 0b1
#define VGA_VPOL_NEGATIVE 0b00
#define VGA_VPOL_POSITIVE 0b10

void vga_writeOCR1A(uint16_t val);
uint16_t vga_readTCNT1();
void vga_init(volatile uint8_t* output, uint16_t width, uint16_t height, uint8_t flags);
void vga_TCIR1A();
void vga_update(uint8_t* buffer);

#endif //VGA_H
