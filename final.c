/*
 * File:   ee3102_final.c
 * Author: Raiyan Mahbub, Kothai Seelan, Erik Haag, Zixin Hu
 *
 * Created on February 23, 2021, 4:17 PM
 */

// PIC24FJ64GA002 Configuration Bit Settings

// 'C' source line config statements

// CONFIG2
#pragma config POSCMOD = HS             // Primary Oscillator Select (HS Oscillator mode selected)
#pragma config I2C1SEL = PRI            // I2C1 Pin Location Select (Use default SCL1/SDA1 pins)
#pragma config IOL1WAY = ON             // IOLOCK Protection (Once IOLOCK is set, cannot be changed)
#pragma config OSCIOFNC = ON            // Primary Oscillator Output Function (OSC2/CLKO/RC15 functions as port I/O (RC15))
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor (Clock switching and Fail-Safe Clock Monitor are disabled)
#pragma config FNOSC = FRCDIV           // Oscillator Select (Fast RC Oscillator with Postscaler (FRCDIV))
#pragma config SOSCSEL = SOSC           // Sec Oscillator Select (Default Secondary Oscillator (SOSC))
#pragma config WUTSEL = LEG             // Wake-up timer Select (Legacy Wake-up Timer)
#pragma config IESO = ON                // Internal External Switch Over Mode (IESO mode (Two-Speed Start-up) enabled)

// CONFIG1
#pragma config WDTPS = PS32768          // Watchdog Timer Postscaler (1:32,768)
#pragma config FWPSA = PR128            // WDT Prescaler (Prescaler ratio of 1:128)
#pragma config WINDIS = ON              // Watchdog Timer Window (Standard Watchdog Timer enabled,(Windowed-mode is disabled))
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (Watchdog Timer is disabled)
#pragma config ICS = PGx1               // Comm Channel Select (Emulator EMUC1/EMUD1 pins are shared with PGC1/PGD1)
#pragma config GWRP = OFF               // General Code Segment Write Protect (Writes to program memory are allowed)
#pragma config GCP = OFF                // General Code Segment Code Protect (Code protection is disabled)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG port is disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdio.h>

#define SIZE 1024

volatile unsigned int circular_buffer[SIZE];    // Initialize circular buffer with SIZE
volatile int front = 0;                         // Write index of circular buffer
volatile int back = 0;                          // Read index of circular buffer
volatile int state = 0;                         // Boolean check if buffer is full or not
volatile int value = 0;                         // Placeholder value in buffer

// ADC Interrupt
void __attribute__((interrupt, auto_psv)) _ADC1Interrupt(void) {
    IFS0bits.AD1IF = 0;                 // Clears interrupt flag
    putVal(ADC1BUF0);                   // Stores latest ADC value in circular buffer
    value = getVal();                   // Takes oldest value from circular buffer
}      

/* Function that stores latest ADC value into circular buffer */
void putVal(int newVal) {
    circular_buffer[front++] = newVal;  // Places new value into buffer, increments front
    if (front == 1024) {                // Checks if buffer is full
        state = 1;
    }
    front &= (SIZE - 1);                // Ensures front index does not exceed buffer size
}

/* Function that sets all circular buffer values to 0 */
int getVal(void) {
    int value;
    value = circular_buffer[back++];    // Gets oldest value in circular buffer
    back &= (SIZE - 1);                 // Ensures back index does not exceed buffer size
    return value;
}

/* Function that clears circular buffer */
void initBuffer(void) {
    int i;
    for(i = 0; i < SIZE; i++) {         // Simple loop that sets all buffer values to 0
        circular_buffer[i] = 0;
    }
}

/* Function to initialize 7-segment LED display */
void initDisplay(void) {
    //AD1PCFG = 0x9fff;                 // Set all pins as digital
    TRISB = 0b0000000001111111;         // Set RB15-RB8 to output
    LATB = 0x03fc;                      // Set RB11 and RB 10 to low and set RB9-RB2 to high
}

/* Function to initialize A/D Converter */
void initADC() {
    CLKDIVbits.RCDIV = 1;
    AD1PCFG = 0x9ffe;
    IFS3bits.MI2C2IF = 0;
    TRISA = 0xFFFF;                     // All PortA Pins: Set as Input
    // LATA = 0x0001;

    AD1CON1 = 0;    
    AD1CON2 = 0;
    AD1CON3 = 0;
    AD1PCFGbits.PCFG5 = 0;              // Input pin AN5/RB3
    
    AD1CON1bits.SSRC = 0b111;           // Starts the conversion
    AD1CON1bits.ASAM = 1;               // Sampling begins immediatly after conversion
    
    AD1CON2bits.VCFG = 0b000;           // Internal Vdd and Vss
    AD1CON2bits.SMPI = 0b0;             // Interrupts at completion every time
    
    AD1CON3bits.ADCS = 0b1;
    AD1CON3bits.SAMC = 0b1;
    
    ADC1BUF0 = 0;
   
    IEC0bits.AD1IE = 1;
    IFS0bits.AD1IF = 0;    
    AD1CON1bits.ADON = 1;

    /*  T3CON = 0;
        TMR3 = 0;
        PR3 = 15625;
        T3CONbits.TCKPS = 0b11;
        T3CONbits.TON = 1;
   
        T3CONbits.TCKPS = 0b11;         //divide by 256, period = 16us
        PR3 = (62500/K)-1;              // Set 1/K seconds duration
       
        T2CON = 0;
        TMR2 = 0;
        PR2 = 25000;
        T2CONbits.TCKPS = 0b10;
        T2CONbits.TON = 1;
       
        IEC0bits.T2IE = 1;
        IFS0bits.T2IF = 0;
    */
}


int main(void) {
    
    initADC();                              // Setup code, initializes: ADC, Display, Buffer
    initDisplay();
    initBuffer();
   
    float lookup[1023] = {};                // Initialize lookup table for differential current
    int i;
    for(i = 0; i < 1024; i++){              // Populate lookup table with values from regression line
        float j = (0.0942 * i) + 4.33;
        lookup[i] = j;
    }

    while(1) {

    }
    
    return 0;
}