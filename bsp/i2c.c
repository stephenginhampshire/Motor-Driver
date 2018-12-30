/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;	Filename:			    i2c.c            
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
#include "xc.h"
#include "i2c.h"

/*********************************************************************
 * Function:        vInitI2C()
 * Input:		None.
 * Output:		None.
 * Overview:		Initialises the I2C(1) peripheral
 * Note:			Sets up Master mode, No slew rate control, 100Khz
 ********************************************************************/
unsigned int vInitI2C(void) {
    //This function will initialise the I2C(1) peripheral.
    //First set the I2C(1) BRG Baud Rate.

    //Consult the dSPIC Data Sheet for information on how to calculate the
    //Baud Rate.

    I2C1BRG = 0x004f;

    //Now we will initialise the I2C peripheral for Master Mode, No Slew Rate
    //Control, and leave the peripheral switched off.

    I2C1CON = 0x1200;

    I2C1RCV = 0x0000;
    I2C1TRN = 0x0000;
    //Now we can enable the peripheral

    I2C1CON = 0x9200;
    return (0);
}

/*********************************************************************
 * Function:        vStartI2C()
 *
 * Input:		None.
 *
 * Output:		None.
 *
 * Overview:		Generates an I2C Start Condition
 *
 * Note:			None
 ********************************************************************/
unsigned int vStartI2C(void) {
    //This function generates an I2C start condition and returns status 
    //of the Start.

    I2C1CONbits.SEN = 1; //Generate Start COndition
    while (I2C1CONbits.SEN); //Wait for Start COndition
    return (I2C1STATbits.S); //Optionally return status
}

/*********************************************************************
 * Function:        vRestartI2C()
 *
 * Input:		None.
 *
 * Output:		None.
 *
 * Overview:		Generates a restart condition and optionally returns status
 *
 * Note:			None
 ********************************************************************/
unsigned int vRestartI2C(void) {
    //This function generates an I2C Restart condition and returns status 
    //of the Restart.

    I2C1CONbits.RSEN = 1; //Generate Restart		
    while (I2C1CONbits.RSEN); //Wait for restart	
    return (I2C1STATbits.S); //Optional - return status
}

/*********************************************************************
 * Function:        vStopI2C()
 *
 * Input:		None.
 *
 * Output:		None.
 *
 * Overview:		Generates a bus stop condition
 *
 * Note:			None
 ********************************************************************/
unsigned int vStopI2C(void) {
    //This function generates an I2C stop condition and returns status 
    //of the Stop.

    I2C1CONbits.PEN = 1; //Generate Stop Condition
    while (I2C1CONbits.PEN); //Wait for Stop
    return (I2C1STATbits.P); //Optional - return status
}

/*********************************************************************
 * Function:        vWriteI2C()
 *
 * Input:		Byte to write.
 *
 * Output:		None.
 *
 * Overview:		Writes a byte out to the bus
 *
 * Note:			None
 ********************************************************************/
unsigned int vWriteI2C(unsigned char byte) {
    //This function transmits the byte passed to the function
    //while (I2C1STATbits.TRSTAT);	//Wait for bus to be idle
    I2C1TRN = byte; //Load byte to I2C1 Transmit buffer
    while (I2C1STATbits.TBF); //wait for data transmission
    return (0);
}

/*********************************************************************
 * Function:        vIdleI2C()
 *
 * Input:		None.
 *
 * Output:		None.
 *
 * Overview:		Waits for bus to become Idle
 *
 * Note:			None
 ********************************************************************/
unsigned int vIdleI2C(void) {
    while (I2C1STATbits.TRSTAT); //Wait for bus Idle
    return (0);
}

/*********************************************************************
 * Function:        vLDByteWriteI2C()
 *
 * Input:		Control Byte, 8 - bit address, data.
 *
 * Output:		None.
 *
 * Overview:		Write a byte to low density device at address LowAdd
 *
 * Note:			None
 ********************************************************************/
unsigned int vLDByteWriteI2C(unsigned char ControlByte, unsigned char LowAdd, unsigned char data) {
    unsigned int ErrorCode;

    vIdleI2C(); //Ensure Module is Idle
    vStartI2C(); //Generate Start COndition
    vWriteI2C(ControlByte); //Write Control byte
    vIdleI2C();

    ErrorCode = vACKStatus(); //Return ACK Status

    vWriteI2C(LowAdd); //Write Low Address
    vIdleI2C();

    ErrorCode = vACKStatus(); //Return ACK Status

    vWriteI2C(data); //Write Data
    vIdleI2C();
    vStopI2C(); //Initiate Stop Condition
    vEEAckPolling(ControlByte); //Perform ACK polling
    return (ErrorCode);
}

/*********************************************************************
 * Function:        vLDByteReadI2C()
 *
 * Input:		Control Byte, Address, *Data, Length.
 *
 * Output:		None.
 *
 * Overview:		Performs a low density read of Length bytes and stores in *Data array
 *				starting at Address.
 *
 * Note:			None
 ********************************************************************/
unsigned int vLDByteReadI2C(unsigned char ControlByte, unsigned char Address, unsigned char *Data, unsigned char Length) {
    vIdleI2C(); //wait for bus Idle
    vStartI2C(); //Generate Start Condition
    vWriteI2C(ControlByte); //Write Control Byte
    vIdleI2C(); //wait for bus Idle
    vWriteI2C(Address); //Write start address
    vIdleI2C(); //wait for bus Idle

    vRestartI2C(); //Generate restart condition
    vWriteI2C(ControlByte | 0x01); //Write control byte for read
    vIdleI2C(); //wait for bus Idle

    vGetsI2C(Data, Length); //read Length number of bytes
    vNotAckI2C(); //Send Not Ack
    vStopI2C(); //Generate Stop
    return (0);
}

/*********************************************************************
 * Function:        vHDByteWriteI2C()
 *
 * Input:		ControlByte, HighAddr, LowAddr, Data.
 *
 * Output:		None.
 *
 * Overview:		perform a high density byte write of data byte, Data.
 *				starting at the address formed from HighAdd and LowAdd
 *
 * Note:			None
 ********************************************************************/
unsigned int vHDByteWriteI2C(unsigned char ControlByte, unsigned char HighAdd, unsigned char LowAdd, unsigned char data) {
    unsigned int ErrorCode;

    vIdleI2C(); //Ensure Module is Idle
    vStartI2C(); //Generate Start COndition
    vWriteI2C(ControlByte); //Write Control byte
    vIdleI2C();

    ErrorCode = vACKStatus(); //Return ACK Status

    vWriteI2C(HighAdd);
    vIdleI2C(); //Write High Address
    vWriteI2C(LowAdd); //Write Low Address
    vIdleI2C();

    ErrorCode = vACKStatus(); //Return ACK Status

    vWriteI2C(data); //Write Data
    vIdleI2C();
    vStopI2C(); //Initiate Stop Condition
    vEEAckPolling(ControlByte); //perform Ack Polling
    return (ErrorCode);
}

/*********************************************************************
 * Function:        vHDByteReadI2C()
 *
 * Input:		Control Byte, HighAdd, LowAdd, *Data, Length.
 *
 * Output:		None.
 *
 * Overview:		Performs a low density read of Length bytes and stores in *Data array
 *				starting at Address formed from HighAdd and LowAdd.
 *
 * Note:			None
 ********************************************************************/
unsigned int vHDByteReadI2C(unsigned char ControlByte, unsigned char HighAdd, unsigned char LowAdd, unsigned char *Data, unsigned char Length) {
    vIdleI2C(); //Wait for bus Idle
    vStartI2C(); //Generate Start condition
    vWriteI2C(ControlByte); //send control byte for write
    vIdleI2C(); //Wait for bus Idle

    vWriteI2C(HighAdd); //Send High Address
    vIdleI2C(); //Wait for bus Idle
    vWriteI2C(LowAdd); //Send Low Address
    vIdleI2C(); //Wait for bus Idle

    vRestartI2C(); //Generate Restart
    vWriteI2C(ControlByte | 0x01); //send control byte for Read
    vIdleI2C(); //Wait for bus Idle

    vGetsI2C(Data, Length); //Read Length number of bytes to Data
    vNotAckI2C(); //send Not Ack
    vStopI2C(); //Send Stop Condition
    return (0);
}

/*********************************************************************
 * Function:        vLDPageWriteI2C()
 *
 * Input:		ControlByte, LowAdd, *wrptr.
 *
 * Output:		None.
 *
 * Overview:		Write a page of data from array pointed to be wrptr
 *				starting at LowAdd
 *
 * Note:			LowAdd must start on a page boundary
 ********************************************************************/
unsigned int vLDPageWriteI2C(unsigned char ControlByte, unsigned char LowAdd, unsigned char *wrptr) {
    vIdleI2C(); //wait for bus Idle
    vStartI2C(); //Generate Start condition
    vWriteI2C(ControlByte); //send controlbyte for a write
    vIdleI2C(); //wait for bus Idle
    vWriteI2C(LowAdd); //send low address
    vIdleI2C(); //wait for bus Idle
    vPutstringI2C(wrptr); //send data
    vIdleI2C(); //wait for bus Idle
    vStopI2C(); //Generate Stop
    return (0);
}

/*********************************************************************
 * Function:        vHDPageWriteI2C()
 *
 * Input:		ControlByte, HighAdd, LowAdd, *wrptr.
 *
 * Output:		None.
 *
 * Overview:		Write a page of data from array pointed to be wrptr
 *				starting at address from HighAdd and LowAdd
 *
 * Note:			Address must start on a page boundary
 ********************************************************************/
unsigned int vHDPageWriteI2C(unsigned char ControlByte, unsigned char HighAdd, unsigned char LowAdd, unsigned char *wrptr) {
    vIdleI2C(); //wait for bus Idle
    vStartI2C(); //Generate Start condition
    vWriteI2C(ControlByte); //send controlbyte for a write
    vIdleI2C(); //wait for bus Idle
    vWriteI2C(HighAdd); //send High Address
    vIdleI2C(); //wait for bus Idle
    vWriteI2C(LowAdd); //send Low Address
    vIdleI2C(); //wait for bus Idle
    vPutstringI2C(wrptr); //Send data
    vIdleI2C(); //wait for bus Idle
    vStopI2C(); //Generate a stop
    return (0);
}

/*********************************************************************
 * Function:        vLDSequentialReadI2C()
 *
 * Input:		ControlByte, address, *rdptr, length.
 *
 * Output:		None.
 *
 * Overview:		Performs a sequential read of length bytes starting at address
 *				and places data in array pointed to by *rdptr
 *
 * Note:			None
 ********************************************************************/
unsigned int vLDSequentialReadI2C(unsigned char ControlByte, unsigned char address, unsigned char *rdptr, unsigned char length) {
    vIdleI2C(); //Ensure Module is Idle
    vStartI2C(); //Initiate start condition
    vWriteI2C(ControlByte); //write 1 byte
    vIdleI2C(); //Ensure module is Idle
    vWriteI2C(address); //Write word address
    vIdleI2C(); //Ensure module is idle
    vRestartI2C(); //Generate I2C Restart Condition
    vWriteI2C(ControlByte | 0x01); //Write 1 byte - R/W bit should be 1 for read
    vIdleI2C(); //Ensure bus is idle
    vGetsI2C(rdptr, length); //Read in multiple bytes
    vNotAckI2C(); //Send Not Ack
    vStopI2C(); //Send stop condition
    return (0);
}

/*********************************************************************
 * Function:        vHDSequentialReadI2C()
 *
 * Input:		ControlByte, HighAdd, LowAdd, *rdptr, length.
 *
 * Output:		None.
 *
 * Overview:		Performs a sequential read of length bytes starting at address
 *				and places data in array pointed to by *rdptr
 *
 * Note:			None
 ********************************************************************/
unsigned int vHDSequentialReadI2C(unsigned char ControlByte, unsigned char HighAdd, unsigned char LowAdd, unsigned char *rdptr, unsigned char length) {
    vIdleI2C(); //Ensure Module is Idle
    vStartI2C(); //Initiate start condition
    vWriteI2C(ControlByte); //write 1 byte
    vIdleI2C(); //Ensure module is Idle
    vWriteI2C(HighAdd); //Write High word address
    vIdleI2C(); //Ensure module is idle
    vWriteI2C(LowAdd); //Write Low word address
    vIdleI2C(); //Ensure module is idle
    vRestartI2C(); //Generate I2C Restart Condition
    vWriteI2C(ControlByte | 0x01); //Write 1 byte - R/W bit should be 1 for read
    vIdleI2C(); //Ensure bus is idle
    vGetsI2C(rdptr, length); //Read in multiple bytes
    vNotAckI2C(); //Send Not Ack
    vStopI2C(); //Send stop condition
    return (0);
}

/*********************************************************************
 * Function:        vACKStatus()
 *
 * Input:		None.
 *
 * Output:		Acknowledge Status.
 *
 * Overview:		Return the Acknowledge status on the bus
 *
 * Note:			None
 ********************************************************************/
unsigned int vACKStatus(void) {
    return (!I2C1STATbits.ACKSTAT); //Return Ack Status
}

/*********************************************************************
 * Function:        vNotAckI2C()
 *
 * Input:		None.
 *
 * Output:		None.
 *
 * Overview:		Generates a NO Acknowledge on the Bus
 *
 * Note:			None
 ********************************************************************/
unsigned int vNotAckI2C(void) {
    I2C1CONbits.ACKDT = 1; //Set for NotACk
    I2C1CONbits.ACKEN = 1;
    while (I2C1CONbits.ACKEN); //wait for ACK to complete
    I2C1CONbits.ACKDT = 0; //Set for NotACk
    return (0);
}

/*********************************************************************
 * Function:        vAckI2C()
 *
 * Input:		None.
 *
 * Output:		None.
 *
 * Overview:		Generates an Acknowledge.
 *
 * Note:			None
 ********************************************************************/
unsigned int vAckI2C(void) {
    I2C1CONbits.ACKDT = 0; //Set for ACk
    I2C1CONbits.ACKEN = 1;
    while (I2C1CONbits.ACKEN); //wait for ACK to complete
    return (0);
}

/*********************************************************************
 * Function:       vgetsI2C()
 *
 * Input:		array pointer, Length.
 *
 * Output:		None.
 *
 * Overview:		read Length number of Bytes into array
 *
 * Note:			None
 ********************************************************************/
unsigned int vGetsI2C(unsigned char *rdptr, unsigned char Length) {
    while (Length--) {
        *rdptr++ = vGetI2C(); //get a single byte

        if (I2C1STATbits.BCL) //Test for Bus collision
        {
            return (-1);
        }

        if (Length) {
            vAckI2C(); //Acknowledge until all read
        }
    }
    return (0);
}

/*********************************************************************
 * Function:        vgetI2C()
 *
 * Input:		None.
 *
 * Output:		contents of I2C1 receive buffer.
 *
 * Overview:		Read a single byte from Bus
 *
 * Note:			None
 ********************************************************************/
unsigned int vGetI2C(void) {
    I2C1CONbits.RCEN = 1; //Enable Master receive
    Nop();
    while (!I2C1STATbits.RBF); //Wait for receive bufer to be full
    return (I2C1RCV); //Return data in buffer
}

/*********************************************************************
 * Function:        vEEAckPolling()
 *
 * Input:		Control byte.
 *
 * Output:		error state.
 *
 * Overview:		polls the bus for an Acknowledge from device
 *
 * Note:			None
 ********************************************************************/
unsigned int vEEAckPolling(unsigned char control) {
    vIdleI2C(); //wait for bus Idle
    vStartI2C(); //Generate Start condition

    if (I2C1STATbits.BCL) {
        return (-1); //Bus collision, return
    }
    else {
        if (vWriteI2C(control)) {
            return (-3); //error return
        }

        vIdleI2C(); //wait for bus idle
        if (I2C1STATbits.BCL) {
            return (-1); //error return
        }

        while (vACKStatus()) {
            vRestartI2C(); //generate restart
            if (I2C1STATbits.BCL) {
                return (-1); //error return
            }

            if (vWriteI2C(control)) {
                return (-3);
            }

            vIdleI2C();
        }
    }
    vStopI2C(); //send stop condition
    if (I2C1STATbits.BCL) {
        return (-1);
    }
    return (0);
}

/*********************************************************************
 * Function:        vputstringI2C()
 *
 * Input:		pointer to array.
 *
 * Output:		None.
 *
 * Overview:		writes a string of data upto PAGESIZE from array
 *
 * Note:			None
 ********************************************************************/
unsigned int vPutstringI2C(unsigned char *wrptr) {
    unsigned char x;

    for (x = 0; x < PAGESIZE; x++) //Transmit Data Until Pagesize
    {
        if (vWriteI2C(*wrptr)) //Write 1 byte
        {
            return (-3); //Return with Write Collision
        }
        vIdleI2C(); //Wait for Idle bus
        if (I2C1STATbits.ACKSTAT) {
            return (-2); //Bus responded with Not ACK
        }
        wrptr++;
    }
    return (0);
}

void read_I2C_buffer(unsigned char addr, unsigned char reg, unsigned char byte_count) {
    //    unsigned char x;
    vStartI2C();
    // **************************
}
