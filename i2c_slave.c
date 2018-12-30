

#include "xc.h"
#include "i2c_slave.h"
#include "bsp/buttons.h"
#include "p33FJ256GP710.h"

union sg_i2c_output_packet {
    unsigned char sg_char[16];
    int sg_int[8];
    long sg_long[4];
};

volatile char I2C_inputbuffer[I2C_BUFFERSIZE + 1];
volatile union sg_i2c_output_packet I2C_outputpacket;
volatile int i2c_input_pointer;
volatile int i2c_output_pointer;

volatile bool i2c_packet_received;

typedef enum {
    STATE_WAITING_FOR_ADDRESS,
    STATE_RECEIVE_DATA_FROM_MASTER,
    STATE_SENDING_DATA_TO_MASTER,
    STATE_SENDING_DATA_LAST
} STATE;

volatile STATE my_i2c_state = STATE_WAITING_FOR_ADDRESS;

void __attribute__((__interrupt__, __no_auto_psv__)) _SI2C1Interrupt(void) {
    unsigned char i2c_rx_char = 0;
    _SI2C1IF = 0; // clear the interrupt flag
    switch (my_i2c_state) {
        case STATE_WAITING_FOR_ADDRESS:
            i2c_rx_char = I2C1RCV; // dummy read to clear buffer full bit
            i2c_input_pointer = 0;
            i2c_output_pointer = 0;
            if (I2C1STATbits.R_W) { // Send data to master
                I2C1TRN = I2C_outputpacket.sg_char[i2c_output_pointer++]; // get first data byte to transmit
                I2C1CONbits.SCLREL = 1; // release the clock so that MASTER can drive it
                my_i2c_state = STATE_SENDING_DATA_TO_MASTER; // change state to sending data to master
            } else { // Receive data from master
                my_i2c_state = STATE_RECEIVE_DATA_FROM_MASTER; // change state to receiving data from master
            }
            break;
        case STATE_RECEIVE_DATA_FROM_MASTER:
            I2C_inputbuffer[i2c_input_pointer++] = I2C1RCV; // save the received character in buffer and increment pointer
            if (i2c_input_pointer >= I2C_BUFFERSIZE) // if last character was a 0 the message is complete
            {
                i2c_packet_received = true;
                my_i2c_state = STATE_WAITING_FOR_ADDRESS; // change state to waiting for address
            }
            break;
        case STATE_SENDING_DATA_TO_MASTER:
            I2CTRN = I2C_outputpacket.sg_char[i2c_output_pointer++];
            I2C1CONbits.SCLREL = 1;
            if (i2c_output_pointer >= I2C_BUFFERSIZE) {
                my_i2c_state = STATE_SENDING_DATA_LAST;
            }
            break;
        case STATE_SENDING_DATA_LAST: //
            my_i2c_state = STATE_WAITING_FOR_ADDRESS;
            break;
        default: // 
            my_i2c_state = STATE_WAITING_FOR_ADDRESS;
    }
}

/******************************************************************************/
void InitI2C(void) {
    I2C1CONbits.I2CEN = 0; // disable I2C
    I2C1CONbits.I2CSIDL = 0;
    I2C1CONbits.SCLREL = 1; //     I2C1CON = 0x1200;
    I2C1CONbits.IPMIEN = 0;
    I2C1CONbits.A10M = 0; // 7 bit address
    I2C1CONbits.DISSLW = 1; // disable slew rate control
    I2C1CONbits.SMEN = 0;
    I2C1CONbits.GCEN = 0; // disable general call
    I2C1CONbits.STREN = 0;
    I2C1CONbits.ACKDT = 0;
    I2C1CONbits.ACKEN = 0;
    I2C1CONbits.RCEN = 0;
    I2C1CONbits.PEN = 0;
    I2C1CONbits.RSEN = 0;
    I2C1CONbits.SEN = 0;

    I2C1BRG = 64; // (40M/400k) - (40M/1.1M)
    I2C1RCV = 0x0000;
    I2C1TRN = 0x0000;
    I2C1CONbits.I2CEN = 1; // enable I2C
    I2C1ADD = SLAVE_I2C_ADDRESS >> 1;
}

void StartI2C(void) {
    I2C1CONbits.SEN = 1;
    while (I2C1CONbits.SEN);
}

void RestartI2C(void) {
    I2C1CONbits.RSEN = 1;
    while (I2C1CONbits.RSEN);
}

void StopI2C(void) {
    I2C1CONbits.PEN = 1;
    while (I2C1CONbits.PEN);
}

unsigned int WriteI2C(unsigned char byte) {
    I2C1TRN = byte;
    while (I2C1STATbits.TBF);
    return (0);
}

void IdleI2C(void) {
    while (I2C1STATbits.TRSTAT);
}

unsigned int LDByteWriteI2C(unsigned char ControlByte, unsigned char LowAdd, unsigned char data) {
    unsigned int ErrorCode;
    IdleI2C();
    StartI2C();
    WriteI2C(ControlByte);
    IdleI2C();
    ErrorCode = ACKStatus();
    WriteI2C(LowAdd);
    IdleI2C();
    ErrorCode = ACKStatus();
    WriteI2C(data);
    IdleI2C();
    StopI2C();
    EEAckPolling(ControlByte);
    return (ErrorCode);
}

unsigned int LDByteReadI2C(unsigned char ControlByte, unsigned char Address, unsigned char *Data, unsigned char Length) {
    IdleI2C();
    StartI2C();
    WriteI2C(ControlByte);
    IdleI2C();
    WriteI2C(Address);
    IdleI2C();
    RestartI2C();
    WriteI2C(ControlByte | 0x01);
    IdleI2C();
    getsI2C(Data, Length);
    NotAckI2C();
    StopI2C();
    return (0);
}

unsigned int HDByteWriteI2C(unsigned char ControlByte, unsigned char HighAdd, unsigned char LowAdd, unsigned char data) {
    unsigned int ErrorCode;

    IdleI2C();
    StartI2C();
    WriteI2C(ControlByte);
    IdleI2C();
    ErrorCode = ACKStatus();
    WriteI2C(HighAdd);
    IdleI2C();
    WriteI2C(LowAdd);
    IdleI2C();
    ErrorCode = ACKStatus();
    WriteI2C(data);
    IdleI2C();
    StopI2C();
    EEAckPolling(ControlByte);
    return (ErrorCode);
}

unsigned int HDByteReadI2C(unsigned char ControlByte, unsigned char HighAdd, unsigned char LowAdd, unsigned char *Data, unsigned char Length) {
    IdleI2C();
    StartI2C();
    WriteI2C(ControlByte);
    IdleI2C();
    WriteI2C(HighAdd);
    IdleI2C();
    WriteI2C(LowAdd);
    IdleI2C();
    RestartI2C();
    WriteI2C(ControlByte | 0x01);
    IdleI2C();
    getsI2C(Data, Length);
    NotAckI2C();
    StopI2C();
    return (0);
}

unsigned int LDPageWriteI2C(unsigned char ControlByte, unsigned char LowAdd, unsigned char *wrptr) {
    IdleI2C();
    StartI2C();
    WriteI2C(ControlByte);
    IdleI2C();
    WriteI2C(LowAdd);
    IdleI2C();
    putstringI2C(wrptr);
    IdleI2C();
    StopI2C();
    return (0);
}

unsigned int HDPageWriteI2C(unsigned char ControlByte, unsigned char HighAdd, unsigned char LowAdd, unsigned char *wrptr) {
    IdleI2C();
    StartI2C();
    WriteI2C(ControlByte);
    IdleI2C();
    WriteI2C(HighAdd);
    IdleI2C();
    WriteI2C(LowAdd);
    IdleI2C();
    putstringI2C(wrptr);
    IdleI2C();
    StopI2C();
    return (0);
}

unsigned int LDSequentialReadI2C(unsigned char ControlByte, unsigned char address, unsigned char *rdptr, unsigned char length) {
    IdleI2C();
    StartI2C();
    WriteI2C(ControlByte);
    IdleI2C();
    WriteI2C(address);
    IdleI2C();
    RestartI2C();
    WriteI2C(ControlByte | 0x01);
    IdleI2C();
    getsI2C(rdptr, length);
    NotAckI2C();
    StopI2C();
    return (0);
}

unsigned int HDSequentialReadI2C(unsigned char ControlByte, unsigned char HighAdd, unsigned char LowAdd, unsigned char *rdptr, unsigned char length) {
    IdleI2C();
    StartI2C();
    WriteI2C(ControlByte);
    IdleI2C();
    WriteI2C(HighAdd);
    IdleI2C();
    WriteI2C(LowAdd);
    IdleI2C();
    RestartI2C();
    WriteI2C(ControlByte | 0x01);
    IdleI2C();
    getsI2C(rdptr, length);
    NotAckI2C();
    StopI2C();
    return (0);
}

unsigned int ACKStatus(void) {
    return (!I2C1STATbits.ACKSTAT);
}

void NotAckI2C(void) {
    I2C1CONbits.ACKDT = 1;
    I2C1CONbits.ACKEN = 1;
    while (I2C1CONbits.ACKEN);
    I2C1CONbits.ACKDT = 0;
}

void AckI2C(void) {
    I2C1CONbits.ACKDT = 0;
    I2C1CONbits.ACKEN = 1;
    while (I2C1CONbits.ACKEN);
}

unsigned int getsI2C(unsigned char *rdptr, unsigned char Length) {
    while (Length--) {
        *rdptr++ = getI2C();
        if (I2C1STATbits.BCL) {
            return (-1);
        }
        if (Length) {
            AckI2C();
        }
    }
    return (0);
}

unsigned int getI2C(void) {
    I2C1CONbits.RCEN = 1;
    Nop();
    while (!I2C1STATbits.RBF);
    return (I2C1RCV);
}

unsigned int EEAckPolling(unsigned char control) {
    IdleI2C();
    StartI2C();
    if (I2C1STATbits.BCL) {
        return (-1);
    } else {
        if (WriteI2C(control)) {
            return (-3);
        }
        IdleI2C();
        if (I2C1STATbits.BCL) {
            return (-1);
        }
        while (ACKStatus()) {
            RestartI2C();
            if (I2C1STATbits.BCL) {
                return (-1);
            }
            if (WriteI2C(control)) {
                return (-3);
            }
            IdleI2C();
        }
    }
    StopI2C();
    if (I2C1STATbits.BCL) {
        return (-1);
    }
    return (0);
}

unsigned int putstringI2C(unsigned char *wrptr) {
    unsigned char x;
    for (x = 0; x < PAGESIZE; x++) {
        if (WriteI2C(*wrptr)) {
            return (-3);
        }
        IdleI2C();
        if (I2C1STATbits.ACKSTAT) {
            return (-2);
        }
        wrptr++;
    }
    return (0);
}