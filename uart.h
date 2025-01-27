/*
 * uart.h
 *
 * Created: 28-07-2024 17:09:18
 *  Author: sumit.chatterjee
 */ 


#ifndef UART_H_
#define UART_H_

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define Baud
#define BaudRate(Baud) (((F_CPU /(Baud * 16UL)))-1)


void UART_init(unsigned int ubrr);          /* Define a BaudRate to Initialize the UART */
unsigned char UART_recieve();               /* UART CHAR Receiver Function */
void UART_transmit(unsigned char data);     /* UART  unsigned CHAR Transmitter Function */
char Read_char();                           /* UART CHAR Read Function */
void Write_char(char val);                  /* UART CHAR Transmitter Function */
unsigned char UART_Available(void);         /* Check UART RECEIVED ANY DATA Function */
void WriteString(char *s);                  /* UART String Transmitter Function */
void Write_dec(uint16_t val);               /* UART DECIMAL Transmitter function- 16 bit */



#endif /* UART_H_ */