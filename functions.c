/*
 * functions.c
 *
 *  Created on: Mar 4, 2023
 *      Author: albtr
 */

#include <msp430.h>
#include <stdlib.h>

#include "functions.h"


void init_gpio(){
    // Configure GPIO For LED1 And LED2
    P1OUT &= ~(BIT0+BIT1);                         // Clear P1.0 output latch for a defined power-on state
    P1DIR |= (BIT0+BIT1);                          // Set P1.0 to output direction

    // Configure GPIO for UART RX and TX pins
    P6SEL1 &= ~(BIT0 | BIT1);
    P6SEL0 |= (BIT0 | BIT1);                // USCI_A3 UART operation


    /* Button GPIO Config
    P5DIR &= ~(BIT5);
    P5REN |= (BIT5);
    P5OUT |= (BIT5);
    P5IES |= (BIT5);
    P5IE |= (BIT5);
    P5IFG &= ~(BIT5); */


    // Limit Switch Config
    P4DIR &= ~(BIT3+BIT2);
    P4REN |= (BIT3+BIT2);
    P4OUT |= (BIT3+BIT2);
    P4IES |= (BIT3+BIT2);
    P4IE |= (BIT3+BIT2);
    P4IFG &= ~(BIT3+BIT2);

    // DRV8424 EVM Configuration Signals
    int fullP3 = BIT1+BIT2+BIT3+BIT4+BIT5+BIT6+BIT7;
    P3OUT &= ~(fullP3);
    P3DIR |= (fullP3);
    P3REN |= (BIT0);
    P3OUT |= (BIT0);


    // DRV8424 EVM Control Signals
    // p5.0 STEP Signal
    // p5.1 DIR Signal
    P5OUT &= ~(BIT0+BIT1);
    P5DIR |= (BIT0+BIT1);

}

//  Sets the Configurations signals for the DRV8424
void init_DRV8424EVM(){
    /*
     * Port 3
     * p3.0 - i - nFault
     * p3.1 - o - m0
     * p3.2 - o - toff
     * p3.3 - o - decay1
     * p3.4 - o - decay0
     * p3.5 - o - m1
     * p3.6 - o - ENABLE
     * p3.7 - o - nSleep
     *
     */

    // Set Microstepping Mode
    //P3OUT &= ~(BIT1);   // M0
    P3OUT |= (BIT1);
    P3OUT &= ~(BIT5);   // M1

    // Set Decay Mode
    P3OUT &= ~(BIT4);   // Decay0
    P3OUT |= (BIT3);    // Decay1

    // Set TOFF
    P3OUT |= (BIT2);    // TOFF

    // Set Operating Mode
    P3OUT &= ~(BIT7);   // nSleep
    P3OUT |= (BIT6);    // ENABLE

}

void init_uart() {
    // Configure USCI_A3 for UART mode
    UCA3CTLW0 = UCSWRST;                    // Put eUSCI in reset
    UCA3CTLW0 |= UCSSEL__SMCLK;             // CLK = SMCLK
    UCA3BRW = 104;                            // 1000000/9600 = 8.68
    UCA3MCTLW = 0x1100;                     // 1000000/9600 - INT(1000000/9600)=0.1670
                                            // UCBRSx value = 0xD6 (See UG)
    UCA3CTLW0 &= ~UCSWRST;                  // release from reset
    UCA3IE |= UCRXIE;                       // Enable USCI_A3 RX interrupt

    rx_i=0;
}

void init_bt(){
    //send_AT();
    //send_AT_NAME();
}

int set_blink_rate(char* arg){
    return atoi(arg);
}

void send_byte(unsigned char txByte){
    while(!(UCA3IFG&UCTXIFG));
    UCA3TXBUF = txByte;
}


void send_AT_NAME(){
    int i = 0;
    unsigned char msg[] = {'A','T','+','N','A','M','E','L','M','S'};
    int btMsgLen = sizeof(msg);

    for(i=0; i < btMsgLen; i++) {
        send_byte(msg[i]);
    }
}

