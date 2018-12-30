//#define ALTITUDE_MOTOR
#define AZIMUTH_MOTOR
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdbool.h>
#include <p33FJ256GP710.h>
#include "spi2.h"
#include "spieeprom.h"
#include <libpic30.h>
#include <math.h>
#include "xc.h"
#include "bsp/lcd.h"
#include "bsp/buttons.h"
#include "bsp/leds.h"
#include "bsp/uart.h"
#include "io_mapping.h"
// #include "ASCOM_Fields.h"
#include "HUB_Commands.h"
// Definitions -----------------------------------------------------------------
#define true    1
#define good    1
#define false   0
#define bad     0
#define error   2
#define FCY                 40000000
#define U1_BAUD_RATE        38400
#define U2_BAUD_RATE        38400
#define BRGVAL_U1           ((FCY/U1_BAUD_RATE)/16)-1
#define BRGVAL_U2           ((FCY/U2_BAUD_RATE)/16)-1
#define STX                 0x02
#define ETX                 0x03
#define DLE                 0x10
#define UPDATE_TIME         500      // number of milliseconds between sending updates to hub
#define AWAY        1
#define HOME        0
#define PRESSED     0
#define NOTPRESSED  1
#define REPLY azimuth
// Pulseguiding Directions -----------------------------------------------------
#define NORTH       0
#define SOUTH       1
#define EAST        2
#define WEST        3
#ifdef ALTITUDE_MOTOR
#define DEGREE2QED   780.6207           // 761.9047619             // QED pulses per degree
#define HOME_BEARING  90.00
#define MAXIMUM_BEARING 180.00
#define SLOW        2815              // PR2 value to achieve speed
#define SIDEREAL    2618
#define LUNAR       2681
#define SOLAR       2625
#define KING        2618
#define PARK        256
#define SLEW        256
#define HOMEING     256
#define TRACKING    2815
#define BACKLASH_0   000            // QED Count - backlash for state 1 CW to CW
#define BACKLASH_1   000            // QED count - backlash for state 2 CW to ACW
#define BACKLASH_2   000            // QED count - backlash for state 3 ACW to CW
#define BACKLASH_3   000            // QED count - backlash for state 4 ACW to ACW
#define CW          1               // clockwise direction
#define ACW         0               // anti clockwise direction
#define SENSE_CW         1               // Home Sensor 2 indicates 1 indicating motor has rotated clockwise
#define SENSE_ACW        0               // Home Sensor 2 indicates 0 indicating motor has rotated anti clockwise
#define TRACKING_DIRECTION 0
#else
#define DEGREE2QED   780.6207             // QED pulses per degree
#define HOME_BEARING   0.00
#define MAXIMUM_BEARING 359.999
#define SLOW        2815              // PR2 value to achieve speed
#define SIDEREAL    2618
#define LUNAR       2681
#define SOLAR       2625
#define KING        2618
#define PARK        275
#define SLEW        275
#define HOMEING     275
#define TRACKING    2815
#define BACKLASH_0   000            // QED Count - backlash for state 1 CW to CW
#define BACKLASH_1   000            // QED count - backlash for state 2 CW to ACW
#define BACKLASH_2   000            // QED count - backlash for state 3 ACW to CW
#define BACKLASH_3   000            // QED count - backlash for state 4 ACW to ACW
#define CW          1               // clockwise direction
#define ACW         0               // anti clockwise direction
#define SENSE_CW    1               // SectorSensor indicates 1 indicating motor has rotated clockwise
#define SENSE_ACW   0               // SectorSensor indicates 0 indicating motor has rotated anti clockwise
#define TRACKING_DIRECTION 1
#endif

//4. ASCOM Values
#define LENGTHMESSAGE  21

/* Function prototypes -------------------------------------------------------*/
void InitPLLIntOsc(void);
void InitApp(void);
void InitIO(void);
void InitCN(void);
void InitOC(void);
void InitTimer1(void);
void InitTimer2(void);
extern void SYS_Initialize(void);
void InitU1(void);
void InitU2(void);
void ClearLEDS(void);
void Motor_Direction(void);
void Check_Buttons(void);
void Check_if_Command_Received(void);
void Send_Update_to_HUB(double, double);
void Send_Reply_to_HUB(char);
void Init32bit_Timer(void);
void Turn_Motor_On(double, char); // takes speed and direction
void Turn_Motor_Off(void);
void Send_Update_to_Display(void);
void Set_Motor_Direction(char);
long Calc_QEDpulses2target(double); // takes bearing and returns QED_pulses and calculated direction
void InitStatus(void);
void Find_Home(void);
void Turn_Motor_On_Guiding(double, char); // takes speed
unsigned long millis(void);
/* Variables -----------------------------------------------------------------*/
unsigned char i;

volatile unsigned char hours = 0;
volatile unsigned char minutes = 0;
volatile unsigned char seconds = 0;
volatile unsigned char millisecondunits = 0;
volatile unsigned char millisecondtens = 0;
volatile unsigned char decimalseconds = 0;
volatile unsigned long millisecond_timer = 0;
volatile unsigned long millissincestart = 0;
volatile unsigned long millispulseguiding = 0;
unsigned long now = 0;

union sg_int_union {
    unsigned char sg_chars[2];
    int sg_int;
};

union sg_long_union {
    unsigned char sg_chars[4];
    long sg_long;
};

union sg_double_union {
    double sg_double;
    char sg_chars[4];
};

typedef struct {
    unsigned char running : 1; // A0
    unsigned char slewing : 1; // A1
    unsigned char tracking : 1; // A2
    unsigned char homeing : 1; // A3
    unsigned char athome : 1; // A4
    unsigned char atpark : 1; // A5
    unsigned char pulseguiding : 1; // A6
    unsigned char change : 1;
} motorstatus;

typedef struct {
    unsigned volatile char dir;
    unsigned volatile int pulse_rising;
    unsigned volatile int pulse_falling;
    unsigned volatile int period;
    volatile double target_speed;
    unsigned volatile int backlash;
} motorparameters;

typedef struct {
    double slow; // 0
    double sidereal; // 1
    double lunar; // 2
    double solar; // 3
    double king; // 4
    double park; // 5
    double slew; // 6
    double homeing; // 7
    double tracking; // 8
} motorspeeds;
motorspeeds Motor_Speeds = {SLOW, SIDEREAL, LUNAR, SOLAR, KING, PARK, SLEW, HOMEING, TRACKING};

typedef struct {
    char command_number; // [1]
    char status_byte; // [2]
    double azimuth_bearing; // [3 - 6]
    double altitude_bearing; // [7 - 10]
    double field3; // [11 - 14]
    double field4; // [15 - 18]
} outgoing_message_structure;
#define outgoing_message_structure_length 20
#define outgoing_message_ETX_position 19

union outgoing_message {
    outgoing_message_structure message;
    unsigned char sg_chars[outgoing_message_structure_length];
};

typedef struct {
    char command_number; // [1]
    char status_byte; // [2]
    double parameter_one; // [3 - 6]
    double parameter_two; // [7 - 10]    
} incoming_message_structure;
#define incoming_message_structure_length 12
#define incoming_message_ETX_position 11

union incoming_message {
    incoming_message_structure message;
    unsigned char sg_chars[incoming_message_structure_length];
};

typedef struct {
    double altitude; // [1 - 4]
    double azimuth; // [5 - 8]
} display_message_structure;
#define display_message_structure_length 10
#define display_message_ETX_position 9

union display_message {
    display_message_structure message;
    unsigned char sg_chars[5];
};

union motorstatusunion {
    motorstatus Motorstatus;
    unsigned char Status;
};
union motorstatusunion MotorStatus;
union motorstatusunion Previous_MotorStatus;
union outgoing_message Outgoing_Message;
union display_message Display_Message;
union incoming_message Incoming_Message;

volatile motorparameters Motordrive;
double azimuth_bearing = 0; // azimuth bearing received via commands
volatile unsigned char U2character;
volatile unsigned char U2Buffer[outgoing_message_structure_length];
volatile unsigned char U2BufferInputPointer = 0;
volatile unsigned char U2BufferOutputPointer = 0;
volatile unsigned char U2ReplyBuffer[outgoing_message_structure_length];
volatile unsigned char U2ReplyBufferInputPointer = 0;
volatile unsigned char U2ReplyBufferOutputPointer = 0;
volatile unsigned char Command_Packet[outgoing_message_structure_length];
volatile bool Incoming_Message_Received = false;
bool Incoming_Message_Reply = false;
char Command_Packet_Reply_answer = 0;
double Command_Packet_Reply_value = 0;
char Command_Packet_Reply_command = 0;
volatile unsigned char U1character = 0;
volatile unsigned char U1Buffer[display_message_structure_length];
volatile unsigned char U1BufferInputPointer = 0;
volatile unsigned char U1BufferOutputPointer = 0;
volatile unsigned char U1ReplyBuffer[display_message_structure_length];
volatile unsigned char U1ReplyBufferInputPointer = 0;
volatile unsigned char U1ReplyBufferOutputPointer = 0;
volatile unsigned char Display_Packet[display_message_structure_length];
volatile bool Display_Packet_Received = false;
// ASCOM values ----------------------------------------------------------------
double TargetBearing_value = 0;
double TargetBearingDirection_value = 0;
double Target_Bearing = 0;
double Slew_Bearing = 0;
volatile long Target_QED = 0;
volatile double Current_QED = 0;
volatile long Movement_QED = 0;
volatile bool Movement_Complete = false;
volatile double Current_Bearing = 0;
volatile double Test_Bearing = 0;
bool Tracking_Rates_Set = false;
volatile double Bearing_Increment = 0; //the amount by which the bearing is increment/decremented for each QED pulse
//-- Pulse Guiding -------------------------------------------------------------
long GuideRateRightAscension = SIDEREAL; // set to default as per ITelescopeV3
long GuideRateDeclination = 0;
int PulseGuide_Direction = 0;
long PulseGuide_Duration = 0;
long Previous_Motor_Speed = 0; // space to store previous rate prior to changing to Pulse Guiding so restore possible
int Previous_Motor_Direction = 0;
volatile long PulseGuide_Speed = 0;
// Interrupt Service Routines --------------------------------------------------

void __attribute__((__interrupt__, __no_auto_psv__)) _T2Interrupt(void) {
    if (MotorStatus.Motorstatus.tracking == true) { // go on forever until cancelled
        if (MotorStatus.Motorstatus.pulseguiding == true) {
            if (millispulseguiding < PulseGuide_Duration) {
                Motordrive.period = PulseGuide_Speed;
            } else {
                MotorStatus.Motorstatus.pulseguiding = false;
                PulseGuide_Duration = 0;
                Motordrive.period = Motor_Speeds.tracking;
            }
        }
        PR2 = Motordrive.period;
        OC1CONbits.OCM = 0b100; // reset Output Compare 
    } else {
        if (MotorStatus.Motorstatus.homeing == true) {
            OC1CONbits.OCM = 0b100; // continue by resetting output compare
        } else { // slewing
            if (Movement_QED != Target_QED) { // current != target 
                if (Motordrive.dir == CW) { // rotation clockwise (Current decrementing)
                    if (Movement_QED > Target_QED) { // current position < target, gone too far so stop
                        OC1CONbits.OCM = 0; // by clearing Output Compare
                        Movement_Complete = true; // and flagging complete
                    } else { // current position >= target position so continue
                        OC1CONbits.OCM = 0b100; // resetting output compare
                    }
                } else { // rotation counter clockwise (Current incrementing))
                    if (Movement_QED < Target_QED) { // current position > target, not gone far enough, so continue
                        OC1CONbits.OCM = 0b100; // by resetting Output Compare
                    } else { // current position <= target so stop
                        OC1CONbits.OCM = 0; // by clearing output compare 
                        Movement_Complete = true; // and flagging complete
                    }
                }
            } else { // current = target
                OC1CONbits.OCM = 0; // so clear output compare
                Movement_Complete = true; // and flagging complete                           
            }
            if (Movement_Complete == true) {
                Turn_Motor_Off();
                MotorStatus.Motorstatus.slewing = false;
            }
        }
    }
    IFS0bits.T2IF = 0; // clear interrupt flag
}

void __attribute__((__interrupt__, __no_auto_psv__)) _OC1Interrupt(void) {
    IFS0bits.OC1IF = 0; // clear the interrupt flag
}

void __attribute__((__interrupt__, __no_auto_psv__)) _T1Interrupt(void) { // millisecond timer
    if (millisecondunits < 9) {
        millisecondunits++;
        millisecond_timer++;
        millissincestart++;
        millispulseguiding++;
    } else {
        millisecondunits = 0;
        if (millisecondtens < 9) {
            millisecondtens++;
        } else {
            millisecondtens = 0;
            if (decimalseconds < 9) {
                decimalseconds++;
            } else {
                decimalseconds = 0;
                if (seconds < 59) // is cumulative seconds < 59?
                {
                    seconds++; // yes, so increment seconds
                } else { // else seconds => 59
                    seconds = 0x00; // reset seconds
                    if (minutes < 59) // is cumulative minutes < 59?
                    {
                        minutes++; // yes, so updates minutes
                    } else { // else minutes => 59
                        minutes = 0x00; // reset minutes
                        if (hours < 23) // is cumulative hours < 23
                        {
                            hours++; // yes, so update hours
                        } else {
                            hours = 0x00; // reset time
                        }
                    }
                }
            }
        }
        IFS0bits.T1IF = 0;
    }
}

void __attribute__((__interrupt__, __no_auto_psv__)) _CNInterrupt(void) {
    static signed short Motor_PreviousRotaryState;
    signed short Motor_CurrentRotaryState;
    LED_Toggle(LED_D8);
    Motor_CurrentRotaryState = ReadRotaryState();
    if (Motordrive.backlash == 0) {
        if (Motor_PreviousRotaryState != Motor_CurrentRotaryState) {
            switch (Motor_CurrentRotaryState) {
                case 0b00: //previous state = 00
                    if (Motor_PreviousRotaryState == 0b10) {
                        Current_QED++;
                        Current_Bearing += Bearing_Increment;
                    }
                    if (Motor_PreviousRotaryState == 0b01) {
                        Current_QED--;
                        Current_Bearing -= Bearing_Increment;
                    }
                    break;
                case 0b01: // previous state RG7 low, RG6 high
                    if (Motor_PreviousRotaryState == 0b00) {
                        Current_QED++;
                        Current_Bearing += Bearing_Increment;
                    }
                    if (Motor_PreviousRotaryState == 0b11) {
                        Current_QED--;
                        Current_Bearing -= Bearing_Increment;
                    }
                    break;
                case 0b11:
                    if (Motor_PreviousRotaryState == 0b01) {
                        Current_QED++;
                        Current_Bearing += Bearing_Increment;
                    }
                    if (Motor_PreviousRotaryState == 0b10) {
                        Current_QED--;
                        Current_Bearing -= Bearing_Increment;
                    }
                    break;
                case 0b10:
                    if (Motor_PreviousRotaryState == 0b11) {
                        Current_QED++;
                        Current_Bearing += Bearing_Increment;
                    }
                    if (Motor_PreviousRotaryState == 0b00) {
                        Current_QED--;
                        Current_Bearing -= Bearing_Increment;
                    }
                    break;
            }
        }
        Motor_PreviousRotaryState = Motor_CurrentRotaryState;
        MotorStatus.Motorstatus.change = true;
        //       if (Current_Bearing >= MAXIMUM_BEARING) Turn_Motor_Off();
        // if (Current_Bearing < (double) 0.0) Current_Bearing = (double) 360.0; // not sure!
        Movement_QED++;
    } else {
        Motordrive.backlash--;
    }
    IFS1bits.CNIF = 0; // clear Input Change Notification Interrupt flag
}

void __attribute__((__interrupt__, __no_auto_psv__)) _U1RXInterrupt(void) {
    unsigned char i = 0;
    if (U1STAbits.FERR == 1);
    if (U1STAbits.OERR == 1);
    U1character = U1RXREG;
    U1Buffer[U1BufferInputPointer++] = U1character;
    if (U1BufferInputPointer > display_message_structure_length) U1BufferInputPointer = 0;

    if ((U1Buffer[display_message_ETX_position] == ETX) && (U1BufferInputPointer == display_message_structure_length) && (U1Buffer[0] == STX)) {
        Display_Packet_Received = true;
        U1Buffer[0] = 0;
        U1Buffer[display_message_ETX_position] = 0; // clear the buffer of control characters
        for (i = 1; i <= display_message_structure_length; i++) { // copy message into storage buffer
            Display_Message.sg_chars[i - 1] = U1Buffer[i];
            U1Buffer[i] = 0; // and clear the buffer
        }
        U1BufferInputPointer = 0;
    }
    _U1RXIF = 0;
}

void __attribute__((__interrupt__, __no_auto_psv__)) _U2RXInterrupt(void) {
    unsigned char i = 0;
    if (U2STAbits.FERR == 1);
    if (U2STAbits.OERR == 1);
    U2character = U2RXREG;
    U2Buffer[U2BufferInputPointer++] = U2character;
    if (U2BufferInputPointer > incoming_message_structure_length) U2BufferInputPointer = 0;

    if ((U2Buffer[incoming_message_ETX_position] == ETX) && (U2BufferInputPointer == incoming_message_structure_length) && (U2Buffer[0] == STX)) {
        Incoming_Message_Received = true;
        U2Buffer[0] = 0;
        U2Buffer[incoming_message_ETX_position] = 0;
        for (i = 1; i <= incoming_message_structure_length; i++) {
            Incoming_Message.sg_chars[i - 1] = U2Buffer[i];
            U2Buffer[i] = 0;
        }
        U2BufferInputPointer = 0;
    }
    IFS1bits.U2RXIF = 0;
}

void __attribute__((__interrupt__, __no_auto_psv__)) _U1TXInterrupt(void) {
    _U1TXIF = 0;
}

void __attribute__((__interrupt__, __no_auto_psv__)) _U2TXInterrupt(void) {
    IFS1bits.U2TXIF = 0;
}
// End of Interrupt Service Routines -------------------------------------------

// Main ------------------------------------------------------------------------

int main(void) {
    SYS_Initialize(); // System initialisation
    printf("\r"); // clear LCD Screen
    ClearLEDS(); // turn all the LEDs off
    InitApp(); // hardware Initialisation
    InitStatus(); // initialise all status bits to false
    MotorStatus.Motorstatus.atpark = true;
    Bearing_Increment = (double) 1 / DEGREE2QED;
    Current_Bearing = HOME_BEARING; // dummy bearing
#ifdef ALTITUDE_MOTOR               // No need to home the Azimuth motor, assume it is pointing north.
    Find_Home(); // find HOME position before everything starts
    while (HomeSensor == AWAY) { // wait until HOME achieved
        if (millisecond_timer >= UPDATE_TIME) {// update timer is incremented every 500 milliseconds
            millisecond_timer = 0;
            Check_if_Command_Received(); //
            Send_Update_to_HUB((double) 0, (double) 0); // send dynamic information to the hub
            Send_Update_to_Display(); // send dynamic information to the 7 Segment display
        } // end of update hub
    } // end of while away, Blocking code until Home reached
#endif
    Turn_Motor_Off();
    Current_Bearing = HOME_BEARING;
    MotorStatus.Motorstatus.homeing = false;
    MotorStatus.Motorstatus.athome = true;
    MotorStatus.Motorstatus.change = true;
    // -- Show some Lights -------------------------------------------------
    LED_On(LED_D4); // At home (on)
    //--------------------------------------------------------------------------
    while (1) {
        LED_Toggle(LED_D3); // Toggle D3 to show programme is running in main loop
        Check_Buttons(); // has operator requested an action by pressing one of the Buttons?               
        Check_if_Command_Received(); // anything received from the hub?
        //-- Update HUB and 7 Segment Display ----------------------------------
        if (millisecond_timer >= UPDATE_TIME) {// update timer is incremented every 500 milliseconds
            millisecond_timer = 0;
            Send_Update_to_HUB((double) Current_QED, (double) Movement_QED); // send dynamic information to the hub
#ifdef ALTITUDE_MOTOR
            Send_Update_to_Display(); // send dynamic information to the 7 Segment display
#endif
        } // end of update hub
    } // end of while
} //end of main ----------------------------------------------------------------

void InitApp(void) {
    InitIO();
    InitPLLIntOsc(); // Initialisation of PLL
    InitTimer1(); // Initialise the clock timer
    InitCN(); // Initialisation of Rotary Encoder with CN module
    InitU1();
    InitU2(); // Initialisation of UART 1
}

void InitIO(void) {
    BUTTON_Enable(BUTTON_1);
    BUTTON_Enable(BUTTON_2);
    BUTTON_Enable(BUTTON_3);
    BUTTON_Enable(BUTTON_4);
    LED_Enable(LED_D3);
    LED_Enable(LED_D4);
    LED_Enable(LED_D5);
    LED_Enable(LED_D6);
    LED_Enable(LED_D7);
    LED_Enable(LED_D8);
    LED_Enable(LED_D9);
    LED_Enable(LED_D10);
    TurnOnMotorDirection(); // initialise the motor direction
    TurnOnHome(); // initialise pins associated with the HOME function
    TurnOnRotary(); // initialise the QEI (Rotary) Encoder)
    TurnOnDisplay(); // initialise the 7 Segment Display
}

void InitStatus(void) {
    MotorStatus.Motorstatus.athome = false;
    MotorStatus.Motorstatus.atpark = false;
    MotorStatus.Motorstatus.homeing = false;
    MotorStatus.Motorstatus.running = false;
    MotorStatus.Motorstatus.slewing = false;
    MotorStatus.Motorstatus.tracking = false;
    MotorStatus.Motorstatus.change = false;
    MotorStatus.Motorstatus.pulseguiding = false;
    MotorStatus.Motorstatus.change = false;
}

void InitTimer1(void) {

    T1CON = 0;
    T1CONbits.TCKPS = 0b10; // 1:64 prescaler
    PR1 = 6250; // every .001 seconds (40,000,000/64/1000)
    IFS0bits.T1IF = 0;
    IPC0bits.T1IP = 3;
    IEC0bits.T1IE = 1;
    T1CONbits.TCS = 0;
    T1CONbits.TON = 1;
}

void InitTimer2(void) {

    T2CONbits.TON = 0; // stop any Timer 2 activity
    T2CONbits.T32 = 0; // Disable 32-bit Timer operation
    T2CONbits.TCS = 0; // Select internal instruction cycle
    T2CONbits.TGATE = 0; // Disable Gated Timer Mode
    T2CONbits.TCKPS = 0; // Select 1:1 prescaler
    TMR2 = 0x00; // clear Timer 2 )lsw)
    PR2 = Motordrive.period; // set required lsw period value
    OC1CONbits.OCTSEL = 0; // select timer 2
    OC1CONbits.OCM = 0; // turn off output compare
    OC1R = Motordrive.pulse_rising; // time for rising edge
    OC1RS = Motordrive.pulse_falling; // time for falling edge
    IPC0bits.OC1IP = 0x05; // set output compare priority
    IFS0bits.OC1IF = 0x0; // clear interrupt flag
    IEC0bits.OC1IE = 0x01; // enable interrupts
    IPC1bits.T2IP = 0x04; // Set Timer 2 Interrupt Priority Level
    IFS0bits.T2IF = 0x00; // Clear Timer 2 Interrupt flag
    IEC0bits.T2IE = 0x01; // Enable Timer 2 interrupt
    //    OC1CONbits.OCM      = 0b100;              // pin starts low, driven high on OC1P/OC1PS
    //    T2CONbits.TON       = 0x01;               // Start Timer
}

void InitCN(void) {
    ADPCFG = 0xffff; //set all pins to digital
    _TRISG6 = 1; // make RG6 pin an input
    _TRISG7 = 1; // make RG7 pin an input
    _CN8IE = 1; // Enable Input Change Notification Interrupt on RG6 pin
    _CN9IE = 1; // Enable Input Change Notification Interrupt on RG7 pin
    _CNIF = 0; // Clear Input Change Notification Flag
    _CNIP = 7; // Set priority of Input Change Notification Interrupt, high so as not to miss changes
    _CNIE = 1; // Enable Input Change Notification Interrupt
}

void InitPLLIntOsc(void) {
    PLLFBD = 38; // M = 43
    CLKDIVbits.PLLPOST = 0; // N1 = 2
    CLKDIVbits.PLLPRE = 0; // N2 = 2
    RCONbits.SWDTEN = 0; // Disable watchdog timer
    __builtin_write_OSCCONH(0x03);
    __builtin_write_OSCCONL(0x01);
    while (OSCCONbits.COSC != 0b011);

    while (OSCCONbits.LOCK != 1);
}

void InitU1(void) {
    U1MODEbits.UARTEN = 1; // Bit15 TX, RX DISABLED, ENABLE at end of func
    //U1MODEbits.not implemented;	// Bit14
    U1MODEbits.USIDL = 0; // Bit13 Continue in Idle
    U1MODEbits.IREN = 0; // Bit12 No IR translation
    U1MODEbits.RTSMD = 0; // Bit11 Simplex Mode
    //U1MODEbits.not implemented;	// Bit10
    U1MODEbits.UEN = 0; // Bits8,9 TX,RX, CTS and RTS enabled
    U1MODEbits.WAKE = 0; // Bit7 No Wake up (since we don't sleep here)
    U1MODEbits.LPBACK = 0; // Bit6 No Loop Back
    U1MODEbits.ABAUD = 0; // Bit5 No Autobaud (would require sending '55')
    U1MODEbits.URXINV = 0; // Bit4 Receive Polarity Inversion (for dsPIC)
    U1MODEbits.BRGH = 0; // Bit3 16 clocks per bit period
    U1MODEbits.PDSEL = 0; // Bits1,2 8bit, No Parity
    U1MODEbits.STSEL = 0; // Bit0 One Stop Bit

    U1BRG = BRGVAL_U1; // set baud rate

    // Load all values in for U2STA SFR
    U1STAbits.UTXISEL1 = 0; //Bit15 Int when Char is transferred (1/2 config!)
    U1STAbits.UTXINV = 0; //Bit14 N/A, IRDA config
    U1STAbits.UTXISEL0 = 0; //Bit13 Other half of Bit15
    //U2STAbits.not implemented = 0;	//Bit12
    U1STAbits.UTXBRK = 0; //Bit11 Disabled
    U1STAbits.UTXEN = 1; //Bit10 TX pins controlled by periph
    U1STAbits.UTXBF = 0; //Bit9 *Read Only Bit*
    U1STAbits.TRMT = 0; //Bit8 *Read Only bit*
    U1STAbits.URXISEL = 0; //Bits6,7 Int. on character received
    U1STAbits.ADDEN = 0; //Bit5 Address Detect Disabled
    U1STAbits.RIDLE = 0; //Bit4 *Read Only Bit*
    U1STAbits.PERR = 0; //Bit3 *Read Only Bit*
    U1STAbits.FERR = 0; //Bit2 *Read Only Bit*
    U1STAbits.OERR = 0; //Bit1 *Read Only Bit*
    U1STAbits.URXDA = 0; //Bit0 *Read Only Bit*

    IPC7 = 0x0300; // U1RXIP Mid Range Interrupt Priority level, no urgent reason

    _U1TXIF = 0; // Clear the Transmit Interrupt Flag
    _U1TXIE = 0; // Enable Transmit Interrupts
    _U1RXIF = 0; // Clear the Receive Interrupt Flag
    _U1RXIE = 1; // Enable Receive Interrupts
    U1MODEbits.UARTEN = 1; // And turn the peripheral on
    U1STAbits.UTXEN = 1;
    for (i = 0; i < outgoing_message_structure_length; i++) {

        U1Buffer[i] = 0;
        U1ReplyBuffer[i] = 0;
    }
}

void InitU2(void) {
    U2MODEbits.UARTEN = 1; // Bit15 TX, RX DISABLED, ENABLE at end of func
    //U1MODEbits.not implemented;	// Bit14
    U2MODEbits.USIDL = 0; // Bit13 Continue in Idle
    U2MODEbits.IREN = 0; // Bit12 No IR translation
    U2MODEbits.RTSMD = 0; // Bit11 Simplex Mode
    //U1MODEbits.not implemented;	// Bit10
    U2MODEbits.UEN = 0; // Bits8,9 TX,RX, CTS and RTS enabled
    U2MODEbits.WAKE = 0; // Bit7 No Wake up (since we don't sleep here)
    U2MODEbits.LPBACK = 0; // Bit6 No Loop Back
    U2MODEbits.ABAUD = 0; // Bit5 No Autobaud (would require sending '55')
    U2MODEbits.URXINV = 0; // Bit4 Receive Polarity Inversion (for dsPIC)
    U2MODEbits.BRGH = 0; // Bit3 16 clocks per bit period
    U2MODEbits.PDSEL = 0; // Bits1,2 8bit, No Parity
    U2MODEbits.STSEL = 0; // Bit0 One Stop Bit

    U2BRG = BRGVAL_U2; //

    // Load all values in for U2STA SFR
    U2STAbits.UTXISEL1 = 0; //Bit15 Int when Char is transferred (1/2 config!)
    U2STAbits.UTXINV = 0; //Bit14 N/A, IRDA config
    U2STAbits.UTXISEL0 = 0; //Bit13 Other half of Bit15
    //U2STAbits.not implemented = 0;	//Bit12
    U2STAbits.UTXBRK = 0; //Bit11 Disabled
    U2STAbits.UTXEN = 1; //Bit10 TX pins controlled by periph
    U2STAbits.UTXBF = 0; //Bit9 *Read Only Bit*
    U2STAbits.TRMT = 0; //Bit8 *Read Only bit*
    U2STAbits.URXISEL = 0; //Bits6,7 Int. on character received
    U2STAbits.ADDEN = 0; //Bit5 Address Detect Disabled
    U2STAbits.RIDLE = 0; //Bit4 *Read Only Bit*
    U2STAbits.PERR = 0; //Bit3 *Read Only Bit*
    U2STAbits.FERR = 0; //Bit2 *Read Only Bit*
    U2STAbits.OERR = 0; //Bit1 *Read Only Bit*
    U2STAbits.URXDA = 0; //Bit0 *Read Only Bit*

    IPC7 = 0x0300; // U1RXIP Mid Range Interrupt Priority level, no urgent reason

    IFS1bits.U2TXIF = 0; // Clear the Transmit Interrupt Flag
    IEC1bits.U2TXIE = 0; // Enable Transmit Interrupts
    IFS1bits.U2RXIF = 0; // Clear the Receive Interrupt Flag
    IEC1bits.U2RXIE = 1; // Enable Receive Interrupts
    U2MODEbits.UARTEN = 1; // And turn the peripheral on
    U2STAbits.UTXEN = 1;
    for (i = 0; i < outgoing_message_structure_length; i++) {

        U2Buffer[i] = 0;
        U2ReplyBuffer[i] = 0;
    }
}

void Turn_Motor_On(double speed, char direction) {
    LED_On(LED_D10);
    Set_Motor_Direction(direction); // implement the motor direction
    Motordrive.period = speed;
    Motordrive.pulse_rising = Motordrive.period >> 1; // OCR - signal goes high
    Motordrive.pulse_falling = Motordrive.period; // OCRS - signal goes low
    InitTimer2();
    Movement_Complete = false;
    Movement_QED = 0;
    OC1CONbits.OCM = 0b100; // pin starts low, driven high on OC1P/OC1PS
    T2CONbits.TON = 0x01; // Start Timer2
    MotorStatus.Motorstatus.running = true;
    MotorStatus.Motorstatus.change = true;
}

void Turn_Motor_On_Guiding(double speed, char direction) {
    //    Set_Motor_Direction(direction);
    Motordrive.period = speed;
    Motordrive.pulse_rising = Motordrive.period >> 1; // OCR - signal goes high
    Motordrive.pulse_falling = Motordrive.period; // OCRS - signal goes low
    InitTimer2();
    OC1CONbits.OCM = 0b100; // pin starts low, driven high on OC1P/OC1PS
    T2CONbits.TON = 0x01; // Start Timer2

}

void Turn_Motor_Off(void) {
    OC1CONbits.OCM = 0; // stop the output compare
    T2CONbits.TON = 0; // stop Timer2
    Movement_QED = 0;
    MotorStatus.Motorstatus.running = false;
    MotorStatus.Motorstatus.change = true;
}

void ClearLEDS(void) {
    LATAbits.LATA1 = 0; // D4
    LATAbits.LATA2 = 0;
    LATAbits.LATA3 = 0;
    LATAbits.LATA4 = 0;
    LATAbits.LATA5 = 0;
    LATAbits.LATA6 = 0;
    LATAbits.LATA7 = 0; // D10
}

void Check_Buttons(void) {
    // Motor Homing - Hardware instruction to Find Home Altitude Motor
    if ((Button_LeftFront == PRESSED) && (Button_RightFront == NOTPRESSED) && (Button_LeftRear == NOTPRESSED) && (Button_RightRear == NOTPRESSED)) {
        if (HomeSensor == AWAY) { // check we are not at home already
            if (SectorSensor == SENSE_ACW) {
                Turn_Motor_On(Motor_Speeds.homeing, CW);
            } else {
                Turn_Motor_On(Motor_Speeds.homeing, ACW);
            }
            MotorStatus.Motorstatus.homeing = true;
            MotorStatus.Motorstatus.athome = false;
            MotorStatus.Motorstatus.change = true;
        }
    }
    // Motor Homing - Hardware instruction to Find Home Azimuth Motor
    if ((Button_LeftFront == NOTPRESSED) && (Button_RightFront == PRESSED) && (Button_LeftRear == NOTPRESSED) && (Button_RightRear == NOTPRESSED)) {
        Send_Reply_to_HUB(AZ_Home_command);
    }
    // Altitude Motor Emergency Stop - Hardware instruction to Stop All Altitude Motor Motion
    if ((Button_LeftFront == NOTPRESSED) && (Button_RightFront == NOTPRESSED) && (Button_LeftRear == PRESSED) && (Button_RightRear == NOTPRESSED)) {
        Turn_Motor_Off();
        MotorStatus.Status = 0; // clear all status flags
        MotorStatus.Motorstatus.change = true;

    }
    // Azimuth Motor Emergency Stop - Hardware instruction to Stop All Azimuth Motor Motion
    if ((Button_LeftFront == NOTPRESSED) && (Button_RightFront == NOTPRESSED) && (Button_LeftRear == NOTPRESSED) && (Button_RightRear == PRESSED)) {
        Send_Reply_to_HUB(AZ_Halt_command);
    }
}

void Check_if_Command_Received(void) {
    if (Incoming_Message_Received == true) {
        LED_Toggle(LED_D7);
        Incoming_Message_Received = false;
        switch (Incoming_Message.sg_chars[0]) { // first byte of packet is the command byte
            case 0:
                break;
                // <editor-fold defaultstate="collapsed" desc="i1 AZ_home_command">
            case AZ_Home_command:
                Send_Reply_to_HUB(AZ_Home_command);
                Find_Home();
                break;
                // </editor-fold>
                // <editor-fold defaultstate="collapsed" desc="i2 AZ_halt_command">
            case AZ_Halt_command:
                Send_Reply_to_HUB(AZ_Halt_command);
                Turn_Motor_Off();
                break;
                // </editor-fold>
                // <editor-fold defaultstate="collapsed" desc="i3 Halt_command">
            case Halt_command:
                break;
                // </editor-fold>
                // <editor-fold defaultstate="collapsed" desc="i4 Azimuth_display">
            case Azimuth_Display_command:
                azimuth_bearing = Incoming_Message.message.parameter_one;
                Send_Reply_to_HUB(Azimuth_Display_command);
                break;
                // </editor-fold>
                // <editor-fold defaultstate="collapsed" desc="c2 TargetAzimuth_command">
            case TargetAzimuth_command:
                TargetBearing_value = Incoming_Message.message.parameter_one;
                Send_Reply_to_HUB(TargetAzimuth_command);
                break;
                // </editor-fold>
                // <editor-fold defaultstate="collapsed" desc="c3 TargetAltitude_command">
            case TargetAltitude_command:
                TargetBearing_value = Incoming_Message.message.parameter_one;
                Send_Reply_to_HUB(TargetAltitude_command);
                break;
                // </editor-fold>
                // <editor-fold defaultstate="collapsed" desc="c4 Tracking_command">
            case Tracking_command: //        Turn tracking on or off
                // some work necessary here to determine speed and direction of track
                if (Incoming_Message.message.parameter_one > (double) 0.0) { // Start
                    if (Tracking_Rates_Set != true) {
                        Motor_Speeds.tracking = Motor_Speeds.sidereal; // default speed = sidereal
                    }
                    Turn_Motor_On(Motor_Speeds.tracking, TRACKING_DIRECTION);
                    MotorStatus.Motorstatus.tracking = true;
                } else { // Stop
                    MotorStatus.Motorstatus.tracking = false;
                    Turn_Motor_Off();
                }
                Send_Reply_to_HUB(Tracking_command);
                break;
                // </editor-fold>
                // <editor-fold defaultstate="collapsed" desc="c5 TrackingRates_command">
            case TrackingRates_command: // variable will contain the speed in arcseconds/second to implement when tracking engaged
                Motor_Speeds.tracking = Incoming_Message.message.parameter_one;
                Tracking_Rates_Set = true;
                Send_Reply_to_HUB(TrackingRates_command);
                break;
                // </editor-fold>
                // <editor-fold defaultstate="collapsed" desc="c6 AbortSlew_command">
            case AbortSlew_command:
                Turn_Motor_Off();
                MotorStatus.Motorstatus.slewing = false;
                Send_Reply_to_HUB(AbortSlew_command);
                break;
                // </editor-fold>
                // <editor-fold defaultstate="collapsed" desc="c7 FindHome_command">
            case FindHome_command:
                Send_Reply_to_HUB(FindHome_command);
                Turn_Motor_Off(); // stop any current movement
                Target_QED = Calc_QEDpulses2target((double) HOME_BEARING); // calculate how far and in which direction to slew 
                Turn_Motor_On(Motor_Speeds.slew, Motordrive.dir); // start the motor turning
                MotorStatus.Motorstatus.slewing = true; // set the status to slewing
                MotorStatus.Motorstatus.homeing = true; // and homeing
                break;
                // </editor-fold>
                // <editor-fold defaultstate="collapsed" desc="c8 Park_command">
            case Park_command:
                Find_Home();
                MotorStatus.Motorstatus.atpark = true;
                Send_Reply_to_HUB(Park_command);
                break;
                // </editor-fold>
                // <editor-fold defaultstate="collapsed" desc="c9 SlewToAltAz_command">
            case SlewToAltAz_command:
                Turn_Motor_Off(); // stop any current movement
                Target_QED = Calc_QEDpulses2target(Incoming_Message.message.parameter_one); // calculate how far and in which direction to slew 
                Turn_Motor_On(Motor_Speeds.slew, Motordrive.dir); // start the motor turning
                MotorStatus.Motorstatus.slewing = true; // set the status to slewing
                Send_Reply_to_HUB(SlewToAltAz_command); // reply to command
                break;
                // </editor-fold>
                // <editor-fold defaultstate="collapsed" desc="c10 SlewToTarget_command">
            case SlewToTarget_command:
                Turn_Motor_Off();
                Target_QED = Calc_QEDpulses2target(TargetBearing_value); // calculate how far and in which direction to slew
                Turn_Motor_On(Motor_Speeds.slew, Motordrive.dir); // start the motor running
                MotorStatus.Motorstatus.slewing = true; // set the status to slewing                
                Send_Reply_to_HUB(SlewToTarget_command);
                break;
                // </editor-fold>
                // <editor-fold defaultstate="collapsed" desc="c11 SyncToAltAz_command">
            case SyncToAltAz_command:
#ifdef AZIMUTH_MOTOR
                Current_Bearing = (double) Incoming_Message.message.parameter_one;
#else
                Current_Bearing = (double) Incoming_Message.message.parameter_one;
#endif                
                Send_Reply_to_HUB(SyncToAltAz_command);
                break;
                // </editor-fold>
                // <editor-fold defaultstate="collapsed" desc="c12 PulseGuide_command">
            case PulseGuide_command:
                PulseGuide_Direction = (int) Incoming_Message.message.parameter_one; // the direction required for the Pulse Guide
                PulseGuide_Duration = (long) Incoming_Message.message.parameter_two; // the required duration, in milliseconds for the Pulse Guide
                switch (PulseGuide_Direction) {
                    case NORTH:
                        PulseGuide_Speed = Motor_Speeds.tracking - GuideRateDeclination; // faster
                        break;
                    case SOUTH:
                        PulseGuide_Speed = Motor_Speeds.tracking + GuideRateDeclination; // slower
                        break;
                    case WEST:
                        PulseGuide_Speed = Motor_Speeds.tracking + GuideRateRightAscension;
                        break;
                    case EAST:
                        PulseGuide_Speed = Motor_Speeds.tracking - GuideRateRightAscension;
                        break;
                }
                millispulseguiding = 0; // zero the pulseguiding timer
                Turn_Motor_On(Motor_Speeds.tracking, TRACKING_DIRECTION);
                MotorStatus.Motorstatus.tracking = true;
                MotorStatus.Motorstatus.pulseguiding = true;
                MotorStatus.Motorstatus.change = true;
                Send_Reply_to_HUB(PulseGuide_command); // CommandMessage.Outgoing_message.field3);
                break;
                // </editor-fold>
                // <editor-fold defaultstate="collapsed" desc="c13 GuideRateRightAscension_command">
            case GuideRateRightAscension_command:
                GuideRateRightAscension = (long) Incoming_Message.message.parameter_one; // the required speed for Pulse Guiding RA, which will be a fraction of SIDEREAL calculated by the .dll
                Send_Reply_to_HUB(GuideRateRightAscension_command);
                break;
                // </editor-fold>
                // <editor-fold defaultstate="collapsed" desc="c14 GuideRateDeclination_command">
            case GuideRateDeclination_command:
                GuideRateDeclination = (long) Incoming_Message.message.parameter_one; // the required speed for Pulse Guiding DEC
                Send_Reply_to_HUB(GuideRateDeclination_command);
                break;
                // </editor-fold>
                // <editor-fold defaultstate="collapsed" desc="c15 UnPark_command">
            case UnPark_command:
                MotorStatus.Motorstatus.atpark = false;
                Send_Reply_to_HUB(UnPark_command);
                break;
                // </editor-fold >
                // <editor-fold defaultstate="collapsed" desc="default_command">
            default:
                break;
                // </editor-fold>
        }
    }
}

void Send_Reply_to_HUB(char command) {
    int i = 0;
    LED_Toggle(LED_D6);
    Outgoing_Message.message.command_number = command; // [0]
    Outgoing_Message.message.status_byte = MotorStatus.Status; // [1]
#ifdef AZIMUTH_MOTOR
    Outgoing_Message.message.azimuth_bearing = (double) Current_Bearing; // [2,3,4,5]
    Outgoing_Message.message.altitude_bearing = 0; // [6,7,8,9]
#else
    Outgoing_Message.message.altitude_bearing = (double) Current_Bearing; // [2,3,4,5]
    Outgoing_Message.message.azimuth_bearing = 0; // [6,7,8,9]
#endif
    Outgoing_Message.message.field3 = (double) 0.0; // [10,11,12,13]
    Outgoing_Message.message.field4 = (double) 0.0; // [14,15,16,17]
    while (U2STAbits.UTXBF);
    U2TXREG = (char) STX;
    for (i = 0; i < 18; i++) {
        while (U2STAbits.UTXBF);
        U2TXREG = Outgoing_Message.sg_chars[i];
    }
    while (U2STAbits.UTXBF);
    U2TXREG = (char) ETX;
}

void Send_Update_to_HUB(double field3, double field4) {
    int i = 0;
    LED_Toggle(LED_D6);
    Outgoing_Message.message.command_number = 0x05; // [0]
    Outgoing_Message.message.status_byte = MotorStatus.Status; // [1]
#ifdef AZIMUTH_MOTOR
    Outgoing_Message.message.azimuth_bearing = (double) Current_Bearing; // [2,3,4,5]
    Outgoing_Message.message.altitude_bearing = 0; // [6,7,8,9]
#else
    Outgoing_Message.message.altitude_bearing = Current_Bearing; // [2,3,4,5]
    Outgoing_Message.message.azimuth_bearing = 0; // [6,7,8,9]
#endif
    Outgoing_Message.message.field3 = (double) field3; // [10,11,12,13]
    Outgoing_Message.message.field4 = (double) field4; // [14,15,16,17]
    while (U2STAbits.UTXBF);
    U2TXREG = (char) STX;
    for (i = 0; i < 18; i++) {
        while (U2STAbits.UTXBF);
        U2TXREG = Outgoing_Message.sg_chars[i];
    }
    while (U2STAbits.UTXBF);
    U2TXREG = (char) ETX;
    MotorStatus.Motorstatus.change = false;
}

void Send_Update_to_Display(void) {
    Display_Message.message.altitude = (double) Current_Bearing; // [0,1,2,3]
    Display_Message.message.azimuth = azimuth_bearing; // 4,5,6,7
    while (U1STAbits.UTXBF);
    U1TXREG = (char) STX;
    for (i = 0; i < 8; i++) {
        while (U1STAbits.UTXBF);
        U1TXREG = Display_Message.sg_chars[i];
    }
    while (U1STAbits.UTXBF);
    U1TXREG = (char) ETX;
}

long Calc_QEDpulses2target(double Target_Bearing) { // Calculate the required number of QED pulses to travel and the direction of travel
    double BearingDelta = (Target_Bearing - Current_Bearing);
    while (BearingDelta <-(double) 180.0) BearingDelta += (double) 360.0;
    while (BearingDelta > (double) 180.0) BearingDelta -= (double) 360.0;
    if (BearingDelta < (double) 0) {
        BearingDelta = (BearingDelta * (double) - 1.0); // BearingDelta should always be positive
        Motordrive.dir = CW;
    } else Motordrive.dir = ACW;
    return (long) (BearingDelta * (double) DEGREE2QED); // calculate and return QED to move
}

void Set_Motor_Direction(char New_Direction) {
    if (Motordrive.dir == CW) {
        if (New_Direction == CW) { // Current = CW new = CW no change, backlash = 0
            Motor_DirectionCW();
            Motordrive.backlash = BACKLASH_0;
        } else { // Current = ACW new =  CW, backlash = 3 
            Motordrive.backlash = BACKLASH_1; // Current = CW, new = ACW, backlash = 1
            Motor_DirectionACW(); // set direction pin to anti clockwise
            Motordrive.dir = ACW;
            LED_On(LED_D9);
            LED_Off(LED_D10);
        }
    } else { // current direction is ACW
        if (New_Direction == ACW) { // Current = ACW new = ACW backlash = 2
            Motordrive.backlash = BACKLASH_2;
            Motor_DirectionACW();
        } else {
            Motordrive.backlash = BACKLASH_3; // current = ACW, new = CW backlash = 3
            Motor_DirectionCW(); // set direction pin to anticlockwise
            Motordrive.dir = CW;
            LED_On(LED_D10);
            LED_Off(LED_D9);
        }
    }
    Motordrive.dir = New_Direction;
}

void Find_Home(void) { // go home
    MotorStatus.Motorstatus.atpark = false;
    if (HomeSensor == HOME) { // are we already at HOME
        MotorStatus.Motorstatus.athome = true; // yes
        Current_Bearing = (double) HOME_BEARING;
        Current_QED = 0;
    } else { // NOT at HOME, so start, or keep motor turning
        if (SectorSensor == SENSE_CW) {
            //            LED_On(LED_D9);
            Turn_Motor_On(Motor_Speeds.homeing, ACW); // start the motor running anti clockwise
        } else {
            //            LED_On(LED_D10);
            Turn_Motor_On(Motor_Speeds.homeing, CW); // start the motor running clockwise
        }
        MotorStatus.Motorstatus.athome = false;
        MotorStatus.Motorstatus.homeing = true;
        MotorStatus.Motorstatus.change = true;
    }
}

unsigned long millis(void) {
    return millissincestart;
}