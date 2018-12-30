/*
 * Using UART2 module
 *
 * File name: uart.h
 * Last modification: 10-04-2013
 */

#ifndef _UART_H
#define _UART_H

/* Speed of the UART2 module */
#define BAUD_RATE_UART2	9600L

/*---------------------------------------------------------------------------
  MACRO: True if the serial ports input buffer contains received data
---------------------------------------------------------------------------*/
#define mIsU2RXDataAvailable() (U2STAbits.URXDA)

/*---------------------------------------------------------------------
  Function name: vInitU2
  Description: Initialisation of UART2 module
  Input parameters: -
  Output parameters: -
-----------------------------------------------------------------------*/
void vInitU2(void);

/*---------------------------------------------------------------------
  Function name: vPutCharU2
  Description: Sends one character to UART2 (use of blocking wait)
  Input parameters: Character to send
  Output parameters: -
-----------------------------------------------------------------------*/
void vPutCharU2(char cChar);

/*---------------------------------------------------------------------
  Function name: vPutStrU2
  Description: Sends string to UART2
  Input parameters: String to send
  Output parameters: -
-----------------------------------------------------------------------*/
void vPutStrU2(char* pcStr);

/*---------------------------------------------------------------------
  Function name: cGetCharU2
  Description: Receives character from UART2 (use of blocking wait)
  Input parameters: -
  Output parameters: Received character
-----------------------------------------------------------------------*/
char cGetCharU2(void);

#endif
