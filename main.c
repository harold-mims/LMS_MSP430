/* --COPYRIGHT--,BSD_EX
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************
 *
 *                       MSP430 CODE EXAMPLE DISCLAIMER
 *
 * MSP430 code examples are self-contained low-level programs that typically
 * demonstrate a single peripheral function or device feature in a highly
 * concise manner. For this the code may rely on the device's power-on default
 * register values and settings such as the clock configuration and care must
 * be taken when combining code from several examples to avoid potential side
 * effects. Also see www.ti.com/grace for a GUI- and www.ti.com/msp430ware
 * for an API functional library-approach to peripheral configuration.
 *
 * --/COPYRIGHT--*/
//******************************************************************************
//  MSP430FR5x9x Demo - USCI_A3 External Loopback test @ 115200 baud
//
//  Description: This demo connects TX to RX of the MSP430 UART
//  The example code shows proper initialization of registers
//  and interrupts to receive and transmit data. If data is incorrect P1.0 LED is
//  turned ON.
//  ACLK = n/a, MCLK = SMCLK = BRCLK = default DCO = 1MHz
//
//
//                MSP430FR5994
//             -----------------
//       RST -|     P6.0/UCA3TXD|----|
//            |                 |    |
//           -|                 |    |
//            |     P6.1/UCA3RXD|----|
//            |                 |
//            |                 |
//            |                 |
//            |    P4.2/LimitSwi|----> Swi --> gnd
//            |    P4.3/LimitSwi|----> Swi --> gnd
//            |                 |
//            |                 |
//            |             P1.0|---> LED
//
//   William Goh
//   Texas Instruments Inc.
//   October 2015
//   Built with IAR Embedded Workbench V6.30 & Code Composer Studio V6.1
//******************************************************************************


/*
 * Important Information for the function of the device
 *
 * The Bluetooth connection is looking for data to be trasnmitted in the format
 * ^A(Type of Signal)(Data)^@
 *
 * Where the Start of Header singal indicates the start of a message, followed by a character indicating the type of signal
 * As of right now the only signal received is 'R' for Roller, followed by the data transmitted ended by a null signal
 *
 *
 * Motor Controller Pins
 *
 * o - MCU 3.3v
 * i - nFault
 * o - Vref/Vrefb/en4
 * o - m0/vrefA/en3
 * o - toff/en3
 * o - decay1/bdecay/en1
 * o - decay0/adecay/mode
 * o - m1/bph/bin2
 * o - STEP/ben/bin1
 * o - DIR/aph/ain2
 * o - ENABLE/aen/ain1
 * o - nsleep
 * o - gnd
 *
 *  We will be using port 3 to send the motor driver configuration signals
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
 * For full Decay Mode see table 7-7 of the datasheet
 *      Smart Tune Ripple Control
 *          decay0 - 0
 *          decay1 - 1
 *
 * Microstepping mode
 *      Full Step
 *          m0-0
 *          m1-0
 *
 *      1/8 Step
 *          m0-1
 *          m1-1
 *
 * TOFF
 *      1 - 19 mA + 2% of ITRIP
 *
 * Operating Modes
 *      Sleep
 *          nsleep = 0
 *
 *      Operating
 *          nSleep = 1
 *          Enable = 1
 *
 *      Disabled
 *          nSleep = 1
 *          Enable = 0
 *
 */
#include <msp430.h>
#include <stdlib.h>

#include <functions.h>


volatile unsigned char RXData = 0;
volatile unsigned char TXData = 1;

unsigned int blinkFreq = 5000;
unsigned int blinkMultiplier = 50;


int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;               // Stop watchdog

    init_gpio();
    init_uart();
    init_bt();
    init_DRV8424EVM();

    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;

    __enable_interrupt();

    for(;;) {
        if (blinkMultiplier < 1) {
            P1OUT &= ~0x01;
            P5OUT &= ~BIT0;
            //P3OUT &= ~(BIT6);    // Sets ENABLE to 0 disabling the motor
        }

        else {
            // Checks to ensure the motor is enabled
            //P3OUT |= (BIT6);    // Sets ENABLE to 1 Enabling the motor
            volatile unsigned int i;            // volatile to prevent optimization
            volatile unsigned int i_max;

            P1OUT ^= BIT0;                      // Toggle P1.0 using exclusive-OR
            P5OUT ^= BIT0;                      // Toggle P1.0 using exclusive-OR


            //i= 156 - (blinkMultiplier) + 1;
            i = 130 - (blinkMultiplier) + 1;
            //i = 5000 - (50 * blinkMultiplier) + 1;
            //i = 500 - (5 * blinkMultiplier) + 1;
            //i_max = i;
            do i--;
            while(i != 0);

        }
    }
}

/*
#pragma vector=PORT5_VECTOR
__interrupt void Port_5(void) {

    //if(P5IN & BIT5 == 1)
    P1OUT ^= BIT1;
    //P1OUT = (P5IN & BIT5);
    P5IFG &= ~BIT5;
}
*/

#pragma vector=PORT4_VECTOR
__interrupt void Port_4(void) {

    if(P4IN == BIT2){
        P1OUT |= BIT1;
        P5OUT |= BIT1;
    }
    else if (P4IN == BIT3) {
        P1OUT &= ~BIT1;
        P5OUT &= ~BIT1;
    }

    //P1OUT ^= BIT1;
    //P1OUT = (P5IN & BIT5);
    P4IFG &= ~(BIT2+BIT3);
}


#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=EUSCI_A3_VECTOR
__interrupt void USCI_A3_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(EUSCI_A3_VECTOR))) USCI_A3_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch(__even_in_range(UCA3IV, USCI_UART_UCTXCPTIFG))
    {
        case USCI_NONE: break;
        case USCI_UART_UCRXIFG:
            while(!(UCA3IFG&UCTXIFG));
            //RXData = UCA3RXBUF;

            rxResponse[rx_i] = UCA3RXBUF;

            if(rx_i > sizeof(rxResponse)-1){
                rx_i=0;
            }

            switch(rxResponse[rx_i]){
                case 0x00:
                    //UCA3TXBUF = UCA3RXBUF;
                    /*
                    if(rxResponse[0] == 'R'){
                        blinkMultiplier = set_blink_rate(&rxResponse[1]);
                    }*/

                    switch(rxResponse[0]) {
                        case 'R':
                            blinkMultiplier = set_blink_rate(&rxResponse[1]);
                            break;
                        case 'C':
                            if(rxResponse[1] == 't'){
                                P3OUT |= (BIT7);    // Sets Sleep to 1 enabling the motor
                                //P1OUT |= BIT1;

                            }
                            else if(rxResponse[1] == 'f') {
                                P3OUT &= ~(BIT7);    // Sets Sleep to 0 disabling the motor
                                //P1OUT &= ~BIT1;

                            }

                    }

                    // Act on Final Char
                case 0x01:
                    rx_i=0;
                    break;
                case 'b':
                    P1OUT ^= BIT1;
                    break;

                default:
                    rx_i++;

            }

            /*
            if(UCA3RXBUF == 'b') {
                P1OUT ^= BIT1;
            }

            if(rxResponse[rx_i] == 0x00){
                UCA3TXBUF = UCA3RXBUF;
                rx_i=0;
            }


            if(('0' <= UCA3RXBUF) && (UCA3RXBUF <= '9')){
                blinkMultiplier = ((int)UCA3RXBUF-48);
            }

            if(rxResponse[rx_i] == 0x01){
                //memset(rxResponse, 0x00, sizeof(rxResponse));
                rx_i=0;
                P1OUT ^= BIT1;
            }

            else {
                rx_i++;
            }
            else {
                UCA3TXBUF = UCA3RXBUF+1;
            }
            */

            __no_operation();
            break;
        case USCI_UART_UCTXIFG: break;
        case USCI_UART_UCSTTIFG: break;
        case USCI_UART_UCTXCPTIFG: break;
        default: break;
    }
}

