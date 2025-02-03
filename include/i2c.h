/*
 * i2c.h
 *
 * Created: 04-08-2024 13:03:10
 *  Author: sumit.chatterjee
 */ 


#ifndef I2C_H_
#define I2C_H_

#include <avr/io.h>
#include "uart.h"
#include <util/delay.h>
#include <avr/sfr_defs.h>
#include <util/twi.h>
#include <compat/twi.h>
#include <stdio.h>
#define ERROR_CODE -1
#ifndef I2C_BAUD
#define I2C_BAUD  1000000UL
#endif
#define I2C_PRE			1		// prescaler i2c
typedef enum{
	err = 0xFF,
	debug = 0x01,
	success = 0x00,
}func_status;


#define bit_get(p,m) ((p) & (m))
#define bit_set(p,m) ((p) |= (m))
#define bit_clear(p,m) ((p) &= ~(m))
#define bit_flip(p,m) ((p) ^= (m))
#define bit_write(c,p,m) (c ? bit_set(p,m) : bit_clear(p,m))
#define BIT(x) (0x01 << (x))
#define LONGBIT(x) ((unsigned long)0x00000001 << (x))

typedef enum {
	TW_FREQ_100K,
	TW_FREQ_250K,
	TW_FREQ_400K
} twi_freq_mode_t;

typedef enum {
	write_mode,
	read_mode,
}twi_Slave_mode;


void tw_init(twi_freq_mode_t twi_freq_mode);
void tw_start(void);
void twMSLA_Init(uint8_t address,twi_Slave_mode mode_v);
void tw_stop(void);
void twMaster_Write(uint8_t data);
void tw_repStart(void);
uint8_t tw_GetStatus(void);
unsigned char tw_read_ACK(void);
unsigned char tw_read_NACK(void);
uint8_t bcdToDec(uint8_t val);
uint8_t decimalToBcd(uint8_t val);
void i2c_scan();
unsigned char i2c_start(void);
unsigned char i2c_repeatStart(void);
unsigned char i2c_sendAddress(unsigned char address);
unsigned char i2c_sendData(unsigned char data);
unsigned char i2c_receiveData_ACK(void);
unsigned char i2c_receiveData_NACK(void);
void i2c_stop(void);
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
uint8_t readByte(uint8_t address, uint8_t subAddress);
void readBytes(uint8_t address, uint8_t subAddress,uint8_t count, uint8_t *dest);




#endif /* I2C_H_ */