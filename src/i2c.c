/*
 * i2c.c
 *
 * Created: 04-08-2024 13:12:48
 *  Author: sumit.chatterjee
 */ 

#include <avr/io.h>
#include "uart.h"
#include "i2c.h"
#include <avr/sfr_defs.h>
#include <util/twi.h>
#include <compat/twi.h>
#include <stdio.h>
#include <string.h>

// Convert normal decimal numbers to binary coded decimal
uint8_t decimalToBcd(uint8_t val){
	return( (val/10*16) + (val%10) );
}

// Convert binary coded decimal to normal decimal numbers
uint8_t bcdToDec(uint8_t val){
	return( (val/16*10) + (val%16) );
}




void tw_init(twi_freq_mode_t twi_freq_mode)
{
	switch (twi_freq_mode)
	{
		case TW_FREQ_100K:
	
		#define I2C_BAUD 100000UL
		TWBR = (((F_CPU /I2C_BAUD) - 16)/(2*I2C_PRE));
       
		break;
		
		case TW_FREQ_250K:
		
		#define I2C_BAUD 250000UL
		TWBR = (((F_CPU /I2C_BAUD) - 16)/(2*I2C_PRE));

		break;
		
		case TW_FREQ_400K:
		
        #define I2C_BAUD 400000UL
		TWBR = (((F_CPU /I2C_BAUD) - 16)/(2*I2C_PRE));

		break;
		default: break;
	}
	Write_dec(TWBR);
	Write_char('\n');
	TWSR = (0<<TWPS1)|(0<<TWPS0);
	TWCR = 0x04;
}



void tw_start(void)
{
	/* Send START condition */
	TWCR =  (1 << TWINT) | (1 << TWEN) | (1 << TWSTA);
	/* Wait for TWINT flag to set */
	while (!(TWCR & (1 << TWINT)));
	/* Check error */
}

void twMSLA_Init(uint8_t address,twi_Slave_mode mode_v)
{
	TWDR = (address << 1) | (mode_v & 0x01);
	/* clear interrupt and enable */
	TWCR = (1 << TWINT | 1 << TWEN );
	/* wait until address transmitted */
	while (!(TWCR & (1 << TWINT)));
}

void tw_stop(void)
{
	/* Send STOP condition */
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
	
}

void twMaster_Write(uint8_t data)
{
	//TWDR = data | (write_mode & 0x01);
	TWDR = data;
	/* clear interrupt and enable */
	TWCR = (1 << TWINT | 1 << TWEN );
	/* wait until address transmitted */
	while (!(TWCR & (1 << TWINT)));
}

unsigned char tw_read_ACK(void)
{
	TWCR = (1 << TWINT) | (1 << TWEN)| (1 << TWEA);
	while(!(TWCR & (1<<TWINT)));
	return TWDR;
}

unsigned char tw_read_NACK(void)
{
	TWCR = (1 << TWINT) | (1 << TWEN);
	while(!(TWCR &(1 << TWINT)));
	return TWDR;
}

void tw_repStart(void)
{

	tw_start();
}
uint8_t tw_GetStatus(void)
{
	uint8_t status;
	//mask status
	status = TWSR & 0xF8;
	return status;
}



//*************************************************
//Function to start i2c communication
//*************************************************
unsigned char i2c_start(void)
{
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN); 	//Send START condition
    while (!(TWCR & (1<<TWINT)));   		//Wait for TWINT flag set. This indicates that the
		  								//START condition has been transmitted
    if ((TWSR & 0xF8) == TW_START)			//Check value of TWI Status Register
 	   return(0);
	else
	   return(1);
}
//*************************************************
//Function for repeat start condition
//*************************************************
unsigned char i2c_repeatStart(void)
{
    TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN); 		//Send START condition
    while (!(TWCR & (1<<TWINT)));   		//Wait for TWINT flag set. This indicates that the
		  								//START condition has been transmitted
    if ((TWSR & 0xF8) == TW_REP_START)			//Check value of TWI Status Register
 	   return(0);
	else
	   return(1);
}
//**************************************************
//Function to transmit address of the slave
//*************************************************
unsigned char i2c_sendAddress(unsigned char address)
{
   unsigned char STATUS;
   if((address & 0x01) == 0) 
     STATUS = TW_MT_SLA_ACK;
   else
     STATUS = TW_MR_SLA_ACK; 
   TWDR = address; 
   TWCR = (1<<TWINT)|(1<<TWEN);	   //Load SLA_W into TWDR Register. Clear TWINT bit
   		  			 				   //in TWCR to start transmission of address
   while (!(TWCR & (1<<TWINT)));	   //Wait for TWINT flag set. This indicates that the
   		 		   					   //SLA+W has been transmitted, and
									   //ACK/NACK has been received.
   if ((TWSR & 0xF8) == STATUS)	   //Check value of TWI Status Register
   	  return(0);
   else 
      return(1);
}
//**************************************************
//Function to transmit a data byte
//*************************************************
unsigned char i2c_sendData(unsigned char data)
{
   TWDR = data; 
   TWCR = (1<<TWINT) |(1<<TWEN);	   //Load SLA_W into TWDR Register. Clear TWINT bit
   		  			 				   //in TWCR to start transmission of data
   while (!(TWCR & (1<<TWINT)));	   //Wait for TWINT flag set. This indicates that the
   		 		   					   //data has been transmitted, and
									   //ACK/NACK has been received.
   if ((TWSR & 0xF8) != TW_MT_DATA_ACK)   //Check value of TWI Status Register
   	  return(1);
   else
      return(0);
}
//*****************************************************
//Function to receive a data byte and send ACKnowledge
//*****************************************************
unsigned char i2c_receiveData_ACK(void)
{
  unsigned char data;
  TWCR = (1<<TWEA)|(1<<TWINT)|(1<<TWEN);
  while (!(TWCR & (1<<TWINT)));	   	   //Wait for TWINT flag set. This indicates that the
   		 		   					   //data has been received
  if ((TWSR & 0xF8) != TW_MR_DATA_ACK)    //Check value of TWI Status Register
   	  return(ERROR_CODE);
  data = TWDR;
  return(data);
}
//******************************************************************
//Function to receive the last data byte (no acknowledge from master
//******************************************************************
unsigned char i2c_receiveData_NACK(void)
{
  unsigned char data;
  TWCR = (1<<TWINT)|(1<<TWEN);
  while (!(TWCR & (1<<TWINT)));	   	   //Wait for TWINT flag set. This indicates that the
   		 		   					   //data has been received
  if ((TWSR & 0xF8) != TW_MR_DATA_NACK)    //Check value of TWI Status Register
   	  return(ERROR_CODE);
  data = TWDR;
  return(data);
}
//**************************************************
//Function to end the i2c communication
//*************************************************   	
void i2c_stop(void)
{
  TWCR =  (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);	  //Transmit STOP condition
}  

void i2c_scan(){
	char tbuf[64]={'\0'};
	for (uint8_t n=0; n<127;n+=1)
	{
		char * adr='\0';
		uint8_t vld=0;
		tw_start();
		if (tw_GetStatus() == TW_START)
		{
			vld+=1;
		}
		twMSLA_Init(n,0);
		if (tw_GetStatus() == TW_MT_SLA_ACK)
		{
			vld+=1;
		}
		tw_stop();
		if (vld == 2)
		{
			WriteString("Address found: ");
			sprintf(tbuf,"0x%x ",n);
			WriteString(tbuf);
			WriteString("\n");
		}
	}
	
}
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data){
	tw_start();
	twMaster_Write((address<<1)|0x00);
	twMaster_Write(subAddress);
	twMaster_Write(data);
	tw_stop();
}
uint8_t readByte(uint8_t address, uint8_t subAddress){
	 uint8_t data;
	 tw_start();
	 twMaster_Write((address<<1)|0x00);
	 twMaster_Write(subAddress);
	 tw_repStart();
	 twMaster_Write((address<<1)|0x01);
	 data=tw_read_NACK();
	 tw_stop();
	 return data;
}

void readBytes(uint8_t address, uint8_t subAddress,uint8_t count, uint8_t *dest){
	tw_start();
	twMaster_Write((address<<1)|0x00);
	twMaster_Write(subAddress);
	tw_repStart();
	
	twMaster_Write((address<<1)|0x01);
	 // Read multiple bytes
	 for (uint8_t i = 0; i < count - 1; i++) {
		 dest[i] = tw_read_ACK(); // Read data with ACK
	 }
	 dest[count - 1] = tw_read_NACK(); // Read last byte with NACK
     i2c_stop();
}  