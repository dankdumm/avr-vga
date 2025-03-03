#ifndef F_CPU
    //8MHz is the built-in clock on the ATMega328p, CLKDIV8 unprogrammed.
    #define F_CPU 8000000
#endif

#include"vga.h"
#include<stdint.h>
#include<avr/interrupt.h>
#include<util/delay.h>

/*
VARIABLE NAMING:
Constants, #defines : named in full uppercase, i.e. SVGA_800X600_60HZ
Static Variables : prefixed with 'vga_', i.e. vga_flags

RELEVANT VGA PINOUT:
PIN 1 - RED
PIN 2 - GREEN
PIN 3 - BLUE

PIN 5 - GND
PIN 6 - RGND
PIN 7 - GGND
PIN 8 - BGND
PIN 9 - +5V VDC
PIN 10 - SYNC GND

PIN 13 - HSYNC
PIN 14 - VSYNC

TIMING:
the frequency of the vga ouput (pixel clock), should ideally be lower than the actual frequency of the CPU clock

FUNCTIONS TO IMPLEMENT:
> init: Uses a byte of data as the output bus. (video bus)
> resolution selection
> video output from a buffer.
> timing? utilise the timer interrupts.

VIDEO BUS FORMAT:
Bit 0: HSYNC
Bit 1: VYSNC
Bit 2: RED
Bit 3: GREEN
Bit 4: BLUE
Bit 5: UNUSED
Bit 6: UNUSED
Bit 7: UNUSED
*/
/*
vgaflags bit definitions:
bit 0: horizontal polarity, 1:positive, 0:negative
bit 1: vertical polarity, 1:positive, 0:negative
bit 2: Compare Match Output, 1:yes, 0:no
bit 3: CS10
bit 4: CS11
bit 5: CS12
bit 6: ClockPixelDivider0
bit 7: ClockPixelDivider1
The clock pixel divider bits are read as if they were a 2-bit number.
*/
/*
vga_state : SYNC SIGNAL STATES: (4 BITS)
xx : indicates the other bits, irrelevant to current info.
xx00: HSYNC DISPLAY
xx01: HYSNC FRONT PORCH
xx10: HYSNC SYNC SIG
xx11: HSYNC BACK PORCH

00xx: VSYNC DISPLAY
01xx: VSYNC FRONT PORCH
10xx: VYSNC SYNC SIG
11xx: VSYNC BACK PORCH
*/
static volatile uint8_t vga_flags = 0;
static volatile uint8_t vga_state = 0;
//vertical count
static volatile uint16_t vga_vcnt = 0;
//vga_video will be used as the output.
static volatile uint8_t* vga_video = 0;

//TODO: incorporate the r/w register functions into 1 function
void vga_writeOCR1A(uint16_t val) {
    //Saving the global interrupt flag
    uint8_t sreg;
    sreg = SREG;
    //Disable interrupts while writing to OCR1A;
    cli();
    //Write to OCR1A
    OCR1A = val;
    //Restore the global interrupt flag
    SREG = sreg;
}

uint16_t vga_readTCNT1() {
    //Saving the global interrupt flag
    uint8_t sreg;
    sreg = SREG;
    uint16_t count;
    //Disable interrupts while reading from TCNT1;
    cli();
    //Read from TCNT1
    count = TCNT1;
    //Restore the global interrupt flag
    SREG = sreg;
    return count;
}

void vga_writeTCNT1(uint16_t val) {
    //Saving the global interrupt flag
    uint8_t sreg;
    sreg = SREG;
    //Disable interrupts while writing to TCNT1;
    cli();
    //Write to TCNT1
    TCNT1 = val;
    //Restore the global interrupt flag
    SREG = sreg;
}

void vga_init(volatile uint8_t* output, uint16_t width, uint16_t height, uint8_t flags) {
    vga_flags = flags;
    vga_video = output;
    //Signal begins at the top left of the screen.
    vga_state = 0;
    *vga_video = 0;
    //Since the signal begins on the DISPLAY section,
    //It (hysnc & vsync) needs to be set based on the sync pulse polarity.
    *vga_video &= (~1) | ~(vga_flags & 1);
    *vga_video &= ~(1<<1) | ~(vga_flags & (1<<1));

    //Set timer interrupt settings.
    //Global Interrupt Enable
    sei();
    //Timer 1, Interrupt Mask Reg, OCIE1A Enabled, allowing for interrupts via timer output compare.
    TIMSK1 |= (1<<1);

    //TCCR = Timer/Counter Control Register.
    //TCCR1A & TCCR1B need to be set for the settings of the timer, clock source, and COM1A; COM1B need to be set to disable Compare Match Output on PB1;
    //Normal Port Operation, OC1A disconnected
    TCCR1A |= (0b00<<6);
    //CS10, CS11, CS12, RESERVED BIT
    //Currently set to "clk/1 no prescaling"
    //TODO: add feature to customize this (Compare Match Output, CS bits)
    TCCR1B |= 0b001;
    TCCR1B &= ~(1<<5);

    //TODO: WRITE TO OCR1A
    vga_writeOCR1A((uint16_t)(F_CPU/1000000)*SVGA_800X600_60HZ_HDIS);
    //vga_writeOCR1A((uint16_t)(F_CPU/1000000)*SVGA_800X600_60HZ_HFPORCH);
}

//Timer Counter Interrupt Routine (1A)
//Is this too slow? we'll see...
void vga_TCIR1A() {
    /*
    The Timer Counter is used to track the HYSNC signal
    A variable, vga_vsync, is used to track the VYSNC signal
    1. Check the vga_state;
    2. Update the state based on the current time and previous state;
    3. Update the OCR1A register;
    4. Update the signal, if needed.
    5. Write the TCNT1 register, if needed.

    Display 0/0
    Front Porch 1/4
    Sync 2/8
    Back Porch 3/12
    */
    //uint16_t vcnt0 = vga_vcnt;
    //hstate
    //TODO: WRITE TO OCR1A
    //TODO: Can probably merge the vga_state and counter together?
    PORTC = 0b00000001;
    //vga_writeTCNT1(0);

    switch(vga_state & 0b11) {
        case 0: //going front porch
            //set hsync pin low(check pol.)
            *vga_video &= (~1) | ~(vga_flags & 1);
            vga_state++;
            vga_writeOCR1A((uint16_t)(F_CPU/1000000)*SVGA_800X600_60HZ_HFPORCH);
            break;

        case 1: //going sync
            //set hysnc pin high (check pol)
            *vga_video &= (~1) | (vga_flags & 1);
            vga_state++;
            vga_writeOCR1A((uint16_t)(F_CPU/1000000)*SVGA_800X600_60HZ_HSYNC);
            break;

        case 2: //going back porch
            //set hsync pin low(check pol)
            *vga_video &= (~1) | ~(vga_flags & 1);
            vga_state++;
            vga_writeOCR1A((uint16_t)(F_CPU/1000000)*SVGA_800X600_60HZ_HBPORCH);
            break;

        case 3: //going display
            //set hsync pin low(check pol)
            *vga_video &= (~1) | ~(vga_flags & 1);
            //reset vga_state bits for hstate
            vga_state &= (0b11<<2);
            vga_vcnt++;
            vga_writeOCR1A((uint16_t)(F_CPU/1000000)*SVGA_800X600_60HZ_HDIS);
            vga_writeTCNT1(0);
            break;
    }

    //vga_vcnt-vcnt0 != 0
    //vstate: check only if the vga_vcnt reaches a boundary
    if(vga_vcnt==SVGA_800X600_60HZ_VDIS ||
        vga_vcnt==SVGA_800X600_60HZ_VFPORCH ||
        vga_vcnt==SVGA_800X600_60HZ_VSYNC ||
        vga_vcnt==SVGA_800X600_60HZ_VBPORCH
        ) {
        switch(vga_state & (0b11<<2)) {
            case 0: //going front porch
                *vga_video &= ~(1<<1) | ~(vga_flags & (1<<1));
                vga_state += 4;
                break;

            case 4: //going sync - high
                *vga_video &= ~(1<<1) | (vga_flags & (1<<1));
                vga_state += 4;
                break;

            case 8: //going back porch
                *vga_video &= ~(1<<1) | ~(vga_flags & (1<<1));
                vga_state += 4;
                break;

            case 12: //going display
                *vga_video &= ~(1<<1) | ~(vga_flags & (1<<1));
                vga_state &= 0b11;
                vga_vcnt = 0;
                break;
        }
    }
}

//TODO: Read up, would the CTC mode or fast PWM mode be suitable for counting instead?
ISR(TIMER1_COMPA_vect, ISR_BLOCK) {
    //Status Register is not preserved.
    vga_TCIR1A();
}

