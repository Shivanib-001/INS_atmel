/*
 * rtc.c
 *
 * Created: 21-08-2024 13:22:27
 *  Author: sumit.chatterjee
 */ 
#include "i2c.h"
#include "uart.h"
#include "rtc.h"
#include <avr/sfr_defs.h>
#include <util/twi.h>
#include <compat/twi.h>

void rtc_init(){
	tw_init(TW_FREQ_400K);
	tw_start();
	twMSLA_Init(0x68,0);
	twMaster_Write(0x07);
	twMaster_Write(0x00);
	tw_stop();
	for (int k =0; k<100; k+=1);
}

void rtc_setTime(unsigned char h, unsigned char m, unsigned char s){
	tw_start();
	twMSLA_Init(0x68,0);
	i2c_sendData(0);
	i2c_sendData(s);
	i2c_sendData(m);
	i2c_sendData(h);
	tw_stop();
	for (int k =0; k<100; k+=1);
}

void Rtc_setTime(uint8_t hs, uint8_t ms, uint8_t ss){
	tw_start();
	twMSLA_Init(0x68,0);
	twMaster_Write(0x00);
	twMaster_Write(ss);
	twMaster_Write(ms);
	twMaster_Write(hs);
	tw_stop();
	for (int k =0; k<100; k+=1);
}


void Rtc_getTime(unsigned char**gsi, unsigned char **gmi,unsigned char **ghi){
	tw_start();
	twMSLA_Init(0x68,0);
	twMaster_Write(0x00);
	for (int k =0; k<100; k+=1);
	tw_repStart();
	twMSLA_Init(0x68,1);
	*gsi = tw_read_ACK();
	*gmi = tw_read_ACK();
	*ghi = tw_read_NACK();
	tw_stop();
}



void rtc_getTime(unsigned char **gs, unsigned char **gm, unsigned char **gh){
	tw_start();
	twMSLA_Init(0x68,0);
	twMaster_Write(0x00);
	for (int k =0; k<100; k+=1);
	tw_repStart();
	twMSLA_Init(0x68,1);
	*gs = i2c_receiveData_ACK();
	*gm = i2c_receiveData_ACK();
	*gh = i2c_receiveData_NACK();
	tw_stop();
}

uint8_t ds3231GetSecond(void)
{
	uint8_t seconds = ds3231_getRegisterValue(DS3231_REGISTER_SECONDS);
	return bcdToDec(seconds);
}

void ds3231_setRegisterPointer(uint8_t reg){
	tw_start();
	twMSLA_Init(0x68,0);
	twMaster_Write(reg);
	tw_stop();
}

uint8_t ds3231_getRegisterValue(uint8_t reg)
{
	ds3231_setRegisterPointer(reg);
	tw_repStart();
	twMSLA_Init(0x68,1);
	uint8_t registerValue = tw_read_ACK();
	tw_stop();

	return registerValue;
}

uint8_t ds3213_write(uint8_t reg,uint8_t values){
	tw_start();
	twMSLA_Init(0x68,0);
	twMaster_Write(reg);
	twMaster_Write(values);
	tw_stop();
}