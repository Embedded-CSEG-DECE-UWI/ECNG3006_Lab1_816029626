/*
 * File:    Lab1_816029626_Q15.c
 * Author:  William Pyke
 * ID No:   816029626
 * 
 * @brief Finite State Machine implemented using Switch Case statements
 * Created on October 3, 2023, 4:38 PM
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

#define WAITING 0       //define FSM states
#define LEDON 1 
#define LEDOFF 2

unsigned int state;
char keyPress = '1';
int tick = 0, receive = 0, ledON = 0, ledOFF = 1;   //define tick int and variable states

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
 *@brief Setup low priority interrupt for recieving data over serial
 *
 *@par
 */
void __interrupt(low_priority) RCint(void)
{
    if (PIR1bits.RCIF && PIE1bits.RCIE)
    {
        RCIF = 0;
        keyPress = RCREG;
        receive = 1;        //sets recieve state to 1, letting states know that a key was pressed
        printf("%c", keyPress);
    }
    return;
}   /* RCint() */

/*!
 *@brief Setup high priority interrupt for counting ticks (timer2 overflow)
 *
 *@par
 */
void __interrupt(high_priority) tmrint(void)
{
    if(PIE1bits.TMR2IE && PIR1bits.TMR2IF)
    {
        PIR1bits.TMR2IF = 0;
        tick++;
    }
    if(tick > 10000)
    {
        tick = 0;
    }
    return;
}   /* tmrint() */

/*!
 *@brief Setup LED pin RD2 for output
 *
 *@par
 */
void setupLED()
{
    TRISDbits.TRISD2 = 0;
    LATDbits.LATD2 = 0;
} /* setupLED() */

/*!
 *@brief Setup Timer 2
 *
 *@par
 */
void timer2setup(void)
{
    T2CON = 0b01111111;
    PIE1bits.TMR2IE = 1;
    PR2 = 0b01001110;
    IPR1bits.TMR2IP = 1;
}   /* timer2setup() */

/*!
 *@brief Setup interrupts
 *
 *@par
 */
void setupINT()
{
    RCONbits.IPEN = 1;
    INTCONbits.GIE = 1;
}   /* setupINT() */

/*!
 *@brief Setup USART 
 *
 *@par
 */
void USARTsetup(void)
{
    TRISCbits.TRISC6 = 1;
    TRISCbits.TRISC7 = 1;
    RCSTAbits.SPEN = 1;
    RCSTA = 0b10110000;
    TXSTAbits.BRGH = 1;
    TXSTAbits.SYNC = 0;
    TXSTAbits.TXEN = 1;
    SPBRGH = 0x00;
    SPBRG = 0b00011001;
    BAUDCON = 0b00000000;
    INTCONbits.PEIE = 1;
    PIE1bits.RCIE = 1;
    IPR1bits.RCIP = 0;
}   /* USARTsetup() */

/*!
 *@brief Wait 500ms or until a unique key is pressed if 500ms has not elapsed
 *
 *@par
 */
void wait()
{
    int now = tick;
    char hold = keyPress;
    while((tick-now)<50)        // wait 500ms
    {
        if(hold != keyPress)    // or break out if a unique key (other than the last one pressed) is pressed
        {
            break;
        }
    }
}   /* wait() */

/*!
 *@brief Turns an LED connected to the RD2 pin on
 *
 *@par
 */    
void LEDon()
{
    LATDbits.LATD2 = 1;
    ledON = 1;
    ledOFF = 0;
    receive = 0;
}   /* LEDon() */

/*!
 *@brief Turns an LED connected to the RD2 pin off
 *
 *@par
 */    
void LEDoff()
{
    LATDbits.LATD2 = 0;
    ledON = 0;
    ledOFF = 1;
    receive = 0;
}   /* LEDoff() */

/*!
 *@brief Main function with FSM logic
 *
 *@par
 */
void main(void)             //The FSM was implemented using Switch Case statments as it was easier than the vector table method,
{                           //but had advantages when compared to the if-else method.
    timer2setup();          //setups various peripherals such as timers, USART, interrups and the LED.
    USARTsetup();
    setupINT();
    setupLED();
    int current = 0;    
	state = 0;
	printf ("\n\n");
    while(1){
        current = tick;
        if((tick-current) > 0)  // wait for the next tick to change state
        {
            //printf("State = %2d \r",state);   would be used for debugging purposes
            switch(state) 
            { 
                case WAITING:                           // In wating state until a character is recieved (keypress)
                    if(receive == 0)
                    {
                        state = WAITING;
                    }
                    else if(ledON == 0 && receive == 1) // switches to LEDON state if key is pressed and LED is off
                    {                                   // note that wait() has the unique key checking functionality
                        state = LEDON; 
                    }
                    else if(ledOFF == 0 && receive == 1)// switches to LEDOFF state if key is pressed and LED is on
                    {
                        state = LEDOFF;
                    }
                break;
                case LEDON:     //sets the LED on, waits 500ms or until unique key pressed and sets the state to WAITING
                    LEDon();
                    wait();
                    state = WAITING;
                break;
                case LEDOFF:    //sets the LED ff, waits 500ms or until unique key pressed and sets the state to WAITING
                    LEDoff();
                    wait();
                    state = WAITING;
                break;
            }
        }
        
    }
    return;
}   /* main() */
