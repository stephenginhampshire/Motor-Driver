/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;                                                                     
;                     Software License Agreement                      
;                                                                     
;     ©2007 Microchip Technology Inc
;     Microchip Technology Inc. ("Microchip") licenses this software to 
;     you solely for the use with Microchip Products. The software is
;     owned by Microchip and is protected under applicable copyright
;     laws. All rights reserved.
;
;     SOFTWARE IS PROVIDED "AS IS." MICROCHIP EXPRESSLY DISCLAIMS ANY
;     WARRANTY OF ANY KIND, WHETHER EXPRESS OR IMPLIED, INCLUDING BUT NOT
;     LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
;     PARTICULAR PURPOSE, OR NON-INFRINGEMENT. IN NO EVENT SHALL MICROCHIP
;     BE LIABLE FOR ANY INCIDENTAL, SPECIAL, INDIRECT OR CONSEQUENTIAL
;     DAMAGES, LOST PROFITS OR LOST DATA, HARM TO YOUR EQUIPMENT, COST OF
;     PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
;     BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF),
;     ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER SIMILAR COSTS.
;     
;                                                                
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;	Filename:			    i2c.h            
;	Date:				    February 21, 2007          
;	File Version:		  	1.0                             
;	Assembled using:		MPLAB IDE 7.51.00.0               
; 	Author:		  	    	Martin Bowman              
;	Company:			    Microchip Technology, Inc.
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

//This file contains the function prototypes for the i2c function
#define PAGESIZE	32

//Low Level Functions
unsigned int vIdleI2C(void);
unsigned int vStartI2C(void);
unsigned int vWriteI2C(unsigned char);
unsigned int vStopI2C(void);
unsigned int vRestartI2C(void);
unsigned int vGetsI2C(unsigned char*, unsigned char);
unsigned int vNotAckI2C(void);
unsigned int vInitI2C(void);
unsigned int vACKStatus(void);
unsigned int vGetI2C(void);
unsigned int vAckI2C(void);
unsigned int vEEAckPolling(unsigned char);
unsigned int vPutstringI2C(unsigned char*);

//High Level Functions for Low Density Devices
unsigned int vLDByteReadI2C(unsigned char, unsigned char, unsigned char*, unsigned char);
unsigned int vLDByteWriteI2C(unsigned char, unsigned char, unsigned char);
unsigned int vLDPageWriteI2C(unsigned char, unsigned char, unsigned char*);
unsigned int vLDSequentialReadI2C(unsigned char, unsigned char, unsigned char*, unsigned char);

//High Level Functions for High Density Devices
unsigned int vHDByteReadI2C(unsigned char, unsigned char, unsigned char, unsigned char*, unsigned char);
unsigned int vHDByteWriteI2C(unsigned char, unsigned char, unsigned char, unsigned char);
unsigned int vHDPageWriteI2C(unsigned char, unsigned char, unsigned char, unsigned char*);
unsigned int vHDSequentialReadI2C(unsigned char, unsigned char, unsigned char, unsigned char*, unsigned char);
