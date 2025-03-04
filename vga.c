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
    //vga_writeOCR1A((uint16_t)(F_CPU/1000000)*SVGA_800X600_60HZ_HDIS);
    //vga_writeOCR1A((uint16_t)(F_CPU/1000000)*(SVGA_HALL_TEST-4));
    //It takes at least 4 cycles for it to enter the ISR, and 4 to return from it
    vga_writeOCR1A((uint16_t)(F_CPU/1000000)*(SVGA_800X600_HLINE));
    vga_writeTCNT1(0);
    //TESTING: OUTPUT GREEN SIGNAL?
    //PORTC |= 0b111<<2;
}

void vga_update() {
    if(vga_vcnt%20==0) PORTC ^= 0b111<<2;
}

//TODO: Read up, would the CTC mode or fast PWM mode be suitable for counting instead?
ISR(TIMER1_COMPA_vect, ISR_BLOCK) {
    //Status Register is not preserved.
    //TODO: NOTE: THE HYSNC TIMING IS STILL FOLLOWING HLINE0, which excludes the BPORCH.
    //TODO: WHY DOES IT SHUTOFF WHEN I CONNECT MORE THAN 1 COLOR LINE?
    //TODO: why is there a 2us delay??
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
    vga_vcnt++;
    if(vga_vcnt>=602 && vga_vcnt<=605) PORTC |= 1<<1;
    else PORTC &= ~(1<<1);
    if(vga_vcnt > 628) vga_vcnt=0;

    //HSYNC SIGNAL
    //if(TCNT1^(uint16_t)(F_CPU/1000000)*SVGA_800X600_HLINE) { PORTC |= 1; PORTC &= ~(0b111<<2); }
    //else { PORTC &= ~1; PORTC |= 0b111<<2; }
    PORTC |= 1; PORTC &= ~(0b111<<2);
    _delay_us(3);
    PORTC &= ~1; PORTC |= 0b111<<2;
    //TCNT1 = TCNT1 - OCR1A;
    TCNT1 = TCNT1 - OCR1A - 3*(uint16_t)(F_CPU/1000000);
}

