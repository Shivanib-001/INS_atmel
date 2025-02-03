#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "rtc.h"
#include "uart.h"
#include "nmea.h"
#include "i2c.h"
#include "bno055.h"
#include <util/delay.h>
#ifndef F_CPU
#define F_CPU 16000000UL
#include <avr/io.h>

#endif

/*------------------------------------------------------------------------------------------------*/
//define variables for gnss data
double va_spd,hypdis, out_spd;
double head_o=0.0;
int kfix=0;
char fdat[32]={'\0'};
bool flag= false;
uint32_t clk_t=0;

//define variables for imu data
float EulCount[3];    // Stores the 16-bit signed Euler angle output
float Pitch, Yaw, Roll;


ISR(TIMER0_OVF_vect){
	clk_t+=1;
	
}
	 
int main(void)
{

    //bno055 initialization:

	tw_init(TW_FREQ_400K);

	i2c_scan();

	OLED_Init();
	rtc_init();

	OLED_Clear();
	OLED_SetCursor(0,0);
    OLED_DisplayString("IMU calibration ...");
	WHO_AM_I();
	
	initBNO055(); // Initialize the BNO055
	WriteStringn("BNO055 initialized for sensor mode....");

	OLED_SetCursor(1,0);
	OLED_DisplayString("all set");
OLED_Clear();

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
	 double magk; 
char * netLat='\0';
	 char * netLng='\0';
	 char * dirLat='\0';
	 char * dirLng='\0';
	 char * netspd='\0';
	 char * timk='\0';
	 char * dati='\0';
	 char * cou='\0';
	 char * magi='\0';
/*
	 char * cou='\0';
	 char * reft='\0';
	 char * coursem='\0';
	 char * refm='\0';
	 char * nspd='\0';
	 char * nunit='\0';
	 char * netspd='\0';
	 char * kunit='\0';
	 char * modevtg='\0';
	 */
	

    float test=0.12;
    while (1) 
    {
        
        readEulData(EulCount); 
		Yaw = EulCount[0];

        sprintf(fdat,"IMU yaw : %.2f \n", Yaw);
		WriteString(fdat);
        OLED_SetCursor(0,0);
		OLED_DisplayString(fdat);
        /*
		        id_rmc(&netLat,&netLng,&dirLat,&dirLng,&netspd,&timk,&dati,&cou,&magi);
            va_lat = atof(netLat);
			va_lng = atof(netLng);
				sprintf(fdat,"lat: %f \n",va_lat);
				OLED_SetCursor(1,0);
				OLED_DisplayString(fdat);
				WriteString(fdat);

				sprintf(fdat,"lon: %f \n",va_lng);
				OLED_SetCursor(2,0);
				OLED_DisplayString(fdat);
				WriteString(fdat);
		*/


	    			

	    id_vtg(&cou,&reft, &coursem,&refm,&nspd,&nunit,&netspd,&kunit,&modevtg);

		out_spd= atof(netspd);
		hypdis = atof(cou);

		//convert to kmph if speed in knots
		// out_spd= 1.852*va_spd;

        if(va_spd!=0){
		    sprintf(fdat,"spd : %f \n", va_spd);
			//WriteString(fdat);
			OLED_SetCursor(1,0);
		    OLED_DisplayString(fdat);

			if (flag==true){
				sprintf(fdat,"heading : %f \n", hypdis);
				//WriteString(fdat);
				OLED_SetCursor(2,0);
		        OLED_DisplayString(fdat);
							
				//yaw = ((hypdis - head_o + 360.0)) % 360;
				yaw = hypdis - head_o;
				if (yaw < 0){
					yaw = yaw + 360.0;
					}
				else if(yaw > 360){
					yaw = yaw - 360.0;
					}
				sprintf(fdat,"yaw :  %f",yaw);
				//WriteString(fdat);
				OLED_SetCursor(3,0);
		        OLED_DisplayString(fdat);
						} 
				
			}

			
			if (va_spd!=0 && hypdis != 0){
				if(flag == false){
					flag = true;
					sprintf(fdat,"heading set at 0 equals %f \n", hypdis);
					//WriteString(fdat);
					OLED_SetCursor(4,0);
					OLED_DisplayString(fdat);
					head_o= hypdis;
				}

			}
			



				}
}



		
				
					
			



