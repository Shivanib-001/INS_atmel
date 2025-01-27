/*
 * uart.c
 *
 * Created: 28-07-2024 17:15:07
 *  Author: sumit.chatterjee
 */ 
#include <avr/io.h>
#include "uart.h"

void UART_init(unsigned int ubrr)
{
	// Set Baud Rate
	UBRR0H = (unsigned char)(ubrr >> 8);
	UBRR0L = (unsigned char)ubrr;
	// Enable Receiver and Transmitter
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	// Set Frame Format
	UCSR0C = (1<<USBS0) | (3<<UCSZ00)|(0<<UMSEL00);
}

void UART_transmit(unsigned char data){
	while(!(UCSR0A & (1<<UDRE0))) {};
	UDR0 = data;
}

unsigned char UART_recieve(){
	while(!(UCSR0A & (1<<RXC0))) {};
	return UDR0;
}

void Write_char(char val){
	while(!(UCSR0A & (1<<UDRE0))) {};
	UDR0 = val;
}

char Read_char(){
	while(!(UCSR0A &(1<<RXC0))) {};
	return UDR0;
}

void WriteString(char *s){
	while (*s)
	{
		Write_char(*s);
		s++;
	}
}

void Write_String(char* StringPtr){
	while(*StringPtr != 0x00){
		Write_char(*StringPtr);
	StringPtr++;}
}

unsigned char UART_Available(void) {
	//return nonzero if char waiting polled version
	unsigned char b;
	b=0;
	if(UCSR0A & (1<<RXC0)) b=1;
	return b;
}

void Write_dec(uint16_t val)
{
	uint8_t dig1 = '0', dig2 = '0', dig3 = '0', dig4 = '0';

	// count value in 10000s place
	while(val >= 10000)
	{
		val -= 10000;
		dig1++;
	}

	// count value in 1000s place
	while(val >= 1000)
	{
		val -= 1000;
		dig2++;
	}

	// count value in 100s place
	while(val >= 100)
	{
		val -= 100;
		dig3++;
	}

	// count value in 10s place
	while(val >= 10)
	{
		val -= 10;
		dig4++;
	}

	// was previous value printed?
	uint8_t prevPrinted = 0;

	// print first digit (or ignore leading zeros)
	if(dig1 != '0') {Write_char(dig1); prevPrinted = 1;}

	// print second digit (or ignore leading zeros)
	if(prevPrinted || (dig2 != '0')) {Write_char(dig2); prevPrinted = 1;}

	// print third digit (or ignore leading zeros)
	if(prevPrinted || (dig3 != '0')) {Write_char(dig3); prevPrinted = 1;}

	// print third digit (or ignore leading zeros)
	if(prevPrinted || (dig4 != '0')) {Write_char(dig4); prevPrinted = 1;}

	// print final digit
	Write_char(val + '0');
}