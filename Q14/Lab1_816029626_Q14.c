/*
 * File:    Lab1_816029626_Q14.c
 * Author:  William Pyke
 * ID No:   816029626
 * 
 * @brief Cyclic Executive Code from William Ch 4, Section 4.7 amended for PIC platform
 * Created on September 27, 2023, 3:08 PM
 */

// PIC18F4620 Configuration Bit Settings
// CONFIG1H
#pragma config OSC = HS         // Oscillator Selection bits (HS oscillator)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (00C000-00FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (00C000-00FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)


#include <xc.h>
#include <stdio.h>
#include <time.h>

#define CLOCKS_PER_SECOND 8000000
#define SLOTX 4
#define CYCLEX 5
#define SLOT_T 5000 // 5 sec slot time
int cycle=0, slot=0;
time_t tick;
clock_t now, then;
struct tm n;

/*!
 *@brief putch function used to transmit char over serial
 *
 *@param[ch] c  Character to be transmitted over serial
 */
void putch(char c)
{
    while(!TXIF)
        continue;
    TXREG = c;
}   /* putch() */

/*!
 *@brief Setup USART 
 *
 *@par
 */
void USARTsetup(void)
{
    TRISCbits.TRISC6 = 1;
    TRISCbits.TRISC7 = 0;
    RCSTAbits.SPEN = 1;
    TXSTAbits.BRGH = 1;
    TXSTAbits.SYNC = 0;
    TXSTAbits.TXEN = 1;
    SPBRGH = 0x00;
    SPBRG = 0b00011001;
    BAUDCON = 0b00000000;
}   /* USARTsetup() */

/*!
 *@brief Setup for TMR0 with 1:8 prescaler and interrupts enabled
 *
 *@par
 */
void timer0setup(void)
{
    T0CON = 0b10000010;
    INTCONbits.GIE = 1;
    INTCONbits.TMR0IE = 1;
}   /* timer0setup() */

/*!
 *@brief ISR handling interrupt for TMR0
 *
 *@par
 */
void __interrupt(high_priority) tcint(void)
{
    if (INTCONbits.TMR0IE && INTCONbits.TMR0IF)
    {
        INTCONbits.TMR0IF = 0;
        tick++;     //increments the tick variable
    }               //running tmr0 continously like this gives the system it's own "real time clock"
    
    return;
}   /* tcint() */

/*!
 *@brief Delay or put the microprocessor to sleep for a set # of seconds
 *
 *@param[in] sec  Number of seconds to sleep
 */
void sleep(int sec)
{
    now = tick;
    while((tick-now)< (sec*4)); //sec*4 since each tick is 250ms (i.e. 1/4 sec)
}   /* sleep() */

/*!
 *@brief Run task 1 and sleep for 1 second
 *
 *@par
 */
void one()                                       
{
    printf("task 1 running\n");
    sleep(1);
}   /* one() */

/*!
 *@brief Run task 2 and sleep for 2 seconds
 *
 *@par
 */
void two() 
{
    printf("task 2 running\n");
    sleep(2);
}   /* two() */

/*!
 *@brief Run task 3 and sleep for 3 seconds
 *
 *@par
 */
void three() 
{
    printf("task 3 running\n");
    sleep(3);
}   /* three() */

/*!
 *@brief Run task 4 and sleep for 4 seconds
 *
 *@par
 */
void four() {
    printf("task 4 running\n");
    sleep(4);
}   /* four() */

/*!
 *@brief Run task 5 and sleep for 5 seconds
 *
 *@par
 */
void five() 
{
    printf("task 5 running\n");
    sleep(5);
}   /* five() */

/*!
 *@brief Run task 5 and sleep for 5 seconds
 *
 *@par
 */
void burn() 
{
    clock_t bstart = tick;
    while (tick < 20) {
            /* burn time here */
    }
    tick = 0;
    printf("burn time = %2.2ldms\n\n", (20-bstart)*
    250);
    cycle = CYCLEX;
}    /* burn() */

/*!
 *@brief Follows a task schedule listed in a vector
 *
 *@param[in] SLOTX      the y-value of the vector
 *@param[in] CYCLEX     the x-value of the vector
 */
void (*ttable[SLOTX] [CYCLEX]) () = 
{
    {one, two, burn, burn, burn},
    {one, three, burn, burn, burn}, 
    {one, four, burn, burn, burn}, 
    {burn, burn, burn, burn, burn} 
};  /* *ttable() */

/*!
 *@brief Main function that setups various peripherals and continuously runs
 *
 *@par
 */
void main(void) 
{
    timer0setup();
    USARTsetup();
    int tps = 4;                        //sets a tick per sec value, in this case 4 since each tick is 250ms
    printf("clock ticks/sec = %d\n\n", tps);
    while (1) {
    for(slot=0; slot<SLOTX; slot++)     //cycles through each slot (y-value)
    for(cycle=0; cycle<CYCLEX; cycle++) //cycles through each item in each slot (x-value)
    (*ttable[slot] [cycle]) ();         // goto next task from table             
}   
return;
}	/* main() */

/*** end of file ***/