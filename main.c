/*
 * multi_point_path.c
 *
 * Created: 24-10-2024 10:50:48
 * Author : sumit.chatterjee
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "rtc.h"
#include "uart.h"
#include "nmea.h"
#ifndef F_CPU
#define F_CPU 16000000UL
#endif


/*------------------------- FUNCTIONS ----------------------------------------------------------*/


/*------------------------------------------------------------------------------------------------*/

double va_spd,hypdis, out_spd;
double head_o=0.0;
int kfix=0;
char fdat[32]={'\0'};
bool flag= false;
uint32_t clk_t=0;

ISR(TIMER0_OVF_vect){
	clk_t+=1;
	
}
	 


int main(void)
{
    /* Replace with your application code */
	 UART_init(BaudRate(9600));

	 /*------------------------- CLOCK GENERATION ---------------------------------*/
	 TCCR0A =0x00;
	 TCCR0B |= (1<<CS01);
	 TIMSK0 |= (1<<TOIE0);
	 sei();
	 /*-----------------------------------------------------------------------------*/
	 PORTD &= (1 << PIND2)| (1 << PIND3) ;
	 
	 char dir_lat;
	 char dir_lng;
	 double va_lng;
	 double va_lat, yaw=0;
	 
	 long int dat,tis;
	 double magk; /*return the float value of Magnetic heading*/
	 /*
	 char * netLat='\0';
	 char * netLng='\0';
	 char * dirLat='\0';
	 char * dirLng='\0';
	 char * netspd='\0';
	 char * timk='\0';
	 char * dati='\0';
	 char * cou='\0';
	 char * magi='\0';*/
	 char * cou='\0';
	 char * reft='\0';
	 char * coursem='\0';
	 char * refm='\0';
	 char * nspd='\0';
	 char * nunit='\0';
	 char * netspd='\0';
	 char * kunit='\0';
	 char * kspd='\0';	


    while (1) 
    {
		//id_rmc(&netLat,&netLng,&dirLat,&dirLng,&netspd,&timk,&dati,&cou,&magi);

	      id_vtg(&coursem,&reft, &cou,&refm,&netspd,&nunit,&nspd,&kunit,&kspd);

			va_spd = atof(netspd);
			hypdis = atof(cou);

			out_spd = 1.852*va_spd;

            if(va_spd!=0){
						sprintf(fdat,"spd : %f \n", va_spd);
						WriteString(fdat);
						if (flag==true){
							sprintf(fdat,"heading : %f \n", hypdis);
							WriteString(fdat);
							
							//yaw = ((hypdis - head_o + 360.0)) % 360;
							yaw = hypdis - head_o;
							if (yaw < 0){
								 yaw = yaw + 360.0;
							}
						    else if(yaw > 360){
							    yaw = yaw - 360.0;
							}
							sprintf(fdat,"yaw :  %f",yaw);
							WriteString(fdat);
						} 
				
			}

			
			if (va_spd!=0 && hypdis != 0){
				if(flag == false){
						flag = true;
						sprintf(fdat,"heading set at 0 equals %f \n", hypdis);
						WriteString(fdat);
						head_o= hypdis;
				}

			}
			



				}
}



		
				
					
			



