/*
 * Using UART module
 *
 * File name: uart.c
 *
 * Last modification: 10-04-2013
 */

#include <xc.h>
#include "uart.h"

#define FCY 40000000ULL //Fcy = (Fosc/2); Fosc = 80MHz

/*---------------------------------------------------------------------
  Function name: vInitUART2
  Description: Initialisation of UART2 module
  Input parameters: -
  Output parameters: -
-----------------------------------------------------------------------*/
void vInitU2(void) {
    U2MODE = 0; // Clear UART1 mode register
    U2STA = 0; // Clear UART1 status register

    U2BRG = (FCY / (16 * BAUD_RATE_UART2)) - 1; // Calculate value of baud rate register


    U2MODEbits.UARTEN = 1; // Enable UART2 module
    U2STAbits.UTXEN = 1; // Enable UART2 transmit 
}

/*---------------------------------------------------------------------
  Function name: vPutCharUART2
  Description: Sends one character to UART2 (use of blocking wait)
  Input parameters: Character to send
  Output parameters: -
-----------------------------------------------------------------------*/
void vPutCharU2(char cChar) {
    while (U2STAbits.UTXBF); // Waits when the output buffer is full
    U2TXREG = cChar; // Puts the character to the buffer
}

/*---------------------------------------------------------------------
  Function name: vPutStrUART2
  Description: Sends string to UART2
  Input parameters: String to send
  Output parameters: -
-----------------------------------------------------------------------*/
void vPutStrU2(char* pcStr) {
    while (*pcStr != 0) {
        vPutCharU2(*pcStr++);
    }
}

/*---------------------------------------------------------------------
  Function name: cGetCharUART2
  Description: Receives character from UART2 (use of blocking wait)
  Input parameters: -
  Output parameters: Received character
-----------------------------------------------------------------------*/
char cGetCharU2(void) {
    while (!U2STAbits.URXDA); // Waits when the input buffer is empty
    return U2RXREG; // Returns with the received character
}
