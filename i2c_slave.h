/* 
 * File:   i2c.h
 * Author: Stephen
 *
 * Created on 25 September 2017, 14:38
 */

#ifndef I2C_DEVICE_H
#define	I2C_DEVICE_H
#include <stdbool.h>

#define PAGESIZE    32
#define true 1
#define false 0
#define SLAVE_I2C_ADDRESS   0x60
#define I2C_BUFFERSIZE    20

void InitI2C(void);
void IdleI2C(void);
void StartI2C(void);
unsigned int WriteI2C(unsigned char);
void StopI2C(void);
void RestartI2C(void);
unsigned int getsI2C(unsigned char*, unsigned char);
unsigned int getI2C(void);
void NotAckI2C(void);
unsigned int ACKStatus(void);
void AckI2C(void);
unsigned int EEAckPolling(unsigned char);
unsigned int putstringI2C(unsigned char*);

unsigned int LDByteReadI2C(unsigned char, unsigned char, unsigned char*, unsigned char);
unsigned int LDByteWriteI2C(unsigned char, unsigned char, unsigned char);
unsigned int LDPageWriteI2C(unsigned char, unsigned char, unsigned char*);
unsigned int LDSequentialReadI2C(unsigned char, unsigned char, unsigned char*, unsigned char);

unsigned int HDByteReadI2C(unsigned char, unsigned char, unsigned char, unsigned char*, unsigned char);
unsigned int HDByteWriteI2C(unsigned char, unsigned char, unsigned char, unsigned char);
unsigned int HDPageWriteI2C(unsigned char, unsigned char, unsigned char, unsigned char*);
unsigned int HDSequentialReadI2C(unsigned char, unsigned char, unsigned char, unsigned char*, unsigned char);




#endif	/* I2C_H */

