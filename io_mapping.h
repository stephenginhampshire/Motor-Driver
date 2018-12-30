/*******************************************************************************
Copyright 2016 Microchip Technology Inc. (www.microchip.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
 *******************************************************************************/

//  RD0 = Pulse pin for servo motor
//  RD1 = Direction pin for servo motor
//
//  RF0
//  RG6 = QEI Channel B
//  RG7 = QEI Channel A
//
//  RD6 = Home Switch
//  RD7 = Home Sensor
//  RD8 = Home Direction Switch

#include "bsp/buttons.h"
#include "bsp/leds.h"
/* LEDs ----------------------------------------------------------------------*/

/* Switches ------------------------------------------------------------------*/
#define BUTTON_1 BUTTON_S3
#define BUTTON_2 BUTTON_S6
#define BUTTON_3 BUTTON_S5
#define BUTTON_4 BUTTON_S4
/* Miscellaneous Inputs ------------------------------------------------------*/
#define TurnOnHome() {_TRISD8=1; _TRISD9=1; _TRISD10=1; _TRISD11=1; _TRISF1=1; _TRISF0=1;}
#define Button_LeftFront    PORTDbits.RD8           // Left Front Button
#define Button_RightFront   PORTDbits.RD9           // Right FrontButton
#define Button_LeftRear     PORTDbits.RD10          // Left Rear Button
#define Button_RightRear    PORTDbits.RD11          // Right Rear Button
#define HomeSensor          PORTFbits.RF1           // At Home Sensor
#define SectorSensor        PORTFbits.RF0           // Segment position sensor, to determine fastest home direction
/* Rotary Encoder ------------------------------------------------------------*/
#define TurnOnRotary() {_TRISG6=1; _TRISG7=1;}
#define ReadRotaryState() (~(PORTG >>6)& 0x0003)   // QEA & QEB pins
/* Motor Control -------------------------------------------------------------*/
/* Pulse state controlled by Output Compare Commands                          */
#define TurnOnMotorDirection() {_LATD1=0; _TRISD1=0;}
#define Motor_DirectionCW()   {_LATD1=1; _TRISD1=0;}    // clockwise = high
#define Motor_DirectionACW()  {_LATD1=0; _TRISD1=0;}    // anticlockwise = low
/* Communications ------------------------------------------------------------*/
#define TurnOnDisplay() {_LATF2=0; _TRISF2=1; _LATF3=0; _TRISF3=0; }

