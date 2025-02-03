/*
 * nmea.c
 *
 * Created: 24-08-2024 13:54:41
 *  Author: sumit.chatterjee
 */

#include "nmea.h"
#include "lcd.h"
#include "uart.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

char fdata[32]={'\0'};
   

void id_rmc(char** lat_a,char** lng_a, char** dr_lat,char** dr_lng,char** spd,char** ctim,char** cdat,char **cors,char **magh){
	uint8_t j=0;
	char * di='\0';
	char rmc_tim[10]={'\0'};
	char rmc_lat[13]={'\0'};
	char rmc_lng[13]={'\0'};
	char rmc_dlat[1]={'\0'};
	char rmc_dlng[1]={'\0'};
	char rmc_spd[10]={'\0'};
	char rmc_crs[10]={'\0'};
	char rmc_ate[11]={'\0'};
	char rmc_mag[10]={'\0'};
	while(UART_Available()>0)
	{
		*di = UART_recieve();
		if (*di == '$')
		{
			*di = UART_recieve();

			if (*di == 'G')
			{
				*di = UART_recieve();

				if (*di =='N')
				{
					*di = UART_recieve();
					if (*di =='R' )
					{
						*di = UART_recieve();
						if (*di == 'M')
						{
							*di = UART_recieve();
							if (*di == 'C')
							{
								*di = UART_recieve();

								if (*di ==',')
								{

									*di = UART_recieve();
									if (*di != ',')
									{
										j=0;
										while(*di !=',' ){
											rmc_tim[j]= *di;
											*di = UART_recieve();
											j+=1;
										}
										*ctim=rmc_tim;
										if (*di == ',')
										{
											*di = UART_recieve();

											if (*di =='A')
											{
												*di = UART_recieve();

												if (*di == ',')
												{
													j=0;
													*di = UART_recieve();

													while (*di !=','){
														rmc_lat[j] = *di;
														*di=UART_recieve();
														j+=1;
													}
													*lat_a = rmc_lat;
													if (*di == ',')
													{
														*di = UART_recieve();
														if (*di == 'N' || *di == 'S'){
															rmc_dlat[0]=*di;
															*di = UART_recieve();
															*dr_lat = rmc_dlat;
															if (*di == ','){
																j=0;
																*di = UART_recieve();
																while(*di !=','){
																	rmc_lng[j] = *di;
																	*di = UART_recieve();
																	j+=1;
																}

																*lng_a = rmc_lng;
																if (*di == ',')
																{
																	*di = UART_recieve();
																	if (*di == 'E' || *di == 'W')
																	{
																		rmc_dlng[0]=*di;
																		*di = UART_recieve();
																		*dr_lng = rmc_dlng;
																		if (*di == ',')
																		{
																			j=0;
																			*di = UART_recieve();
																			while (*di !=',')
																			{
																				rmc_spd[j] = *di;
																				*di = UART_recieve();
																				j+=1;
																			}
																			*spd = rmc_spd;
																			if (*di == ',')
																			{
																				j=0;
																				*di = UART_recieve();

																				while (*di !=',')
																				{
																					rmc_crs[j]= *di;
																					*di = UART_recieve();

																					j+=1;
																				}
																				*cors = rmc_crs;
																				if (*di == ',')
																				{
																					j=0;
																					*di = UART_recieve();

																					while (*di !=',')
																					{
																						rmc_ate[j]= *di;
																						*di = UART_recieve();
																						j+=1;
																					}
																					*cdat=rmc_ate;
																					if (*di == ',')
																					{
																						j=0;
																						*di = UART_recieve();

																						while (*di !=',')
																						{
																							rmc_mag[j]= *di;
																							*di = UART_recieve();
																							j+=1;
																						}
																						*magh = rmc_mag;

																					}
																				}
																			}

																		}
																	}
																}
															}
														}
													}

												}
											} else if (*di == 'V')
											{
												Write_String("RMC -- Invalid");
												OLED_SetCursor(0,0);
												OLED_DisplayString("RMC --Invalid");
											}
										}
									}
									else if (*di ==',')
									{
										*di = UART_recieve();
										if (*di == 'V')
										{
											Write_String("INVALID \n");
											OLED_SetCursor(0,0);
											OLED_DisplayString("Signal --Invalid");
										}

									}

								}
							}
						}
					}
				}
				else if(*di =='P')
				{
					*di = UART_recieve();

					if (*di =='R' )
					{
						*di = UART_recieve();
						if (*di == 'M')
						{
							*di = UART_recieve();
							if (*di == 'C')
							{
								*di = UART_recieve();

								if (*di ==',')
								{

									*di = UART_recieve();
									if (*di != ',')
									{
										j=0;
										while(*di !=',' ){
											rmc_tim[j]= *di;
											*di = UART_recieve();
											j+=1;
										}
										*ctim=rmc_tim;
										if (*di == ',')
										{
											*di = UART_recieve();

											if (*di =='A')
											{
												*di = UART_recieve();

												if (*di == ',')
												{
													j=0;
													*di = UART_recieve();

													while (*di !=','){
														rmc_lat[j] = *di;
														*di=UART_recieve();
														j+=1;
													}
													*lat_a = rmc_lat;
													if (*di == ',')
													{
														*di = UART_recieve();
														if (*di == 'N' || *di == 'S'){
															rmc_dlat[0]=*di;
															*di = UART_recieve();
															*dr_lat = rmc_dlat;
															if (*di == ','){
																j=0;
																*di = UART_recieve();
																while(*di !=','){
																	rmc_lng[j] = *di;
																	*di = UART_recieve();
																	j+=1;
																}
																*lng_a = rmc_lng;
																if (*di == ',')
																{
																	*di = UART_recieve();
																	if (*di == 'E' || *di == 'W')
																	{
																		rmc_dlng[0]=*di;
																		*di = UART_recieve();
																		*dr_lng = rmc_dlng;
																		if (*di == ',')
																		{
																			j=0;
																			*di = UART_recieve();
																			while (*di !=',')
																			{
																				rmc_spd[j] = *di;
																				*di = UART_recieve();
																				j+=1;
																			}
																			*spd = rmc_spd;

																			if (*di == ',')
																			{
																				j=0;
																				*di = UART_recieve();

																				while (*di !=',')
																				{
																					rmc_crs[j]= *di;
																					*di = UART_recieve();

																					j+=1;
																				}

																				if (*di == ',')
																				{
																					j=0;
																					*di = UART_recieve();

																					while (*di !=',')
																					{
																						rmc_ate[j]= *di;
																						*di = UART_recieve();
																						j+=1;
																					}
																					*cdat = rmc_ate;
																					if (*di == ',')
																					{
																						j=0;
																						*di = UART_recieve();

																						while (*di !=',')
																						{
																							rmc_mag[j]= *di;
																							*di = UART_recieve();
																							j+=1;
																						}


																					}
																				}
																			}

																		}
																	}
																}
															}
														}
													}

												}
											} else if (*di == 'V')
											{
												Write_String("RMC -- Invalid");

											}
										}
									}
									else if (*di ==',')
									{
										*di = UART_recieve();
										if (*di == 'V')
										{
											Write_String("INVALID \n");
										}

									}

								}
							}
						}
					}


				}
			}
		}
	}
}



double ConvertDegtoRad(double degree) {
	double pi = 3.14159265359;
	return (degree * (pi /180));
}


double ConvertRadtoDeg(double radians) {
	double pi = 3.14159265359;
	return (radians * (180 /pi));
}


double lat_filt(float def) {
	double latitude = 0.0;
	float k_lat_deg=(def*0.01);
	unsigned int deg = (int)k_lat_deg;
	if(deg > 8 && 37 > deg){
		float sec = (def- (float)deg*100)/60;
		latitude = (float)deg + sec;
	}

	return latitude;
}



double lng_filt(float kef){
	double longitude = 0.0;
	float k_lng_deg=(kef*0.01);
	unsigned int deglng = (int)k_lng_deg;
	if(deglng > 68 && 97 > deglng){
		float seclng = (kef- (float)deglng*100)/60;
		longitude = (float)deglng + seclng;

	}

	return longitude;
}

void convert_time_to_UTC(unsigned long int UTC_Time) {

	unsigned int hour, min, sec;

	hour = (UTC_Time / 10000);
	min = (UTC_Time % 10000) / 100;
	sec = (UTC_Time % 10000) % 100;
	hour = hour+5;
	if (hour >= 23)
	{
		hour = 23-hour;

	}
	min = min + 30;
	if (min > 59)
	{
		min = min-60;
		//hour++;
	}

	sprintf(fdata,"Time: %02d:%02d:%02d \n", hour, min, sec);
	Write_String(fdata);
	OLED_SetCursor(7,0);
	OLED_DisplayString(fdata);

}

void id_gga(char** glat_a,char** glng_a, char** dg_lat,char** dg_lng,char** ghdop,char** gtim,char** galt,char** gdalt,char** gfix_ty){
	uint8_t t=0;
	char * ki='\0';
	char gga_lat[13]={'\0'};
	char gga_tim[10]={'\0'};
	char gga_lng[13]={'\0'};
	char gga_dlat[1]={'\0'};
	char gga_dlng[1]={'\0'};
	char gga_alt[7]={'\0'};
	char gga_dalt[1]={'\0'};
	char fix_ty[2]={'\0'};
	char nsat[2]={'\0'};
	char hdop[3]={'\0'};

	while(UART_Available()>0)
	{
		*ki = UART_recieve();
		if (*ki == '$')
		{
			*ki = UART_recieve();

			if (*ki == 'G')
			{
				*ki = UART_recieve();

				if (*ki =='N')
				{
					*ki = UART_recieve();
					if (*ki =='G' )
					{
						*ki = UART_recieve();
						if (*ki == 'G')
						{
							*ki = UART_recieve();
							if (*ki == 'A')
							{
								*ki = UART_recieve();

								if (*ki ==',')
								{

									*ki = UART_recieve();
									if (*ki != ',')
									{
										t=0;
										while(*ki !=',' ){
											gtim[t]= *ki;
											*ki = UART_recieve();
											t+=1;
										}
										//*gtim = gga_tim;

										if (*ki == ',')
										{
											t=0;
											*ki = UART_recieve();
											while (*ki !=','){
												gga_lat[t] = *ki;
												*ki=UART_recieve();
												t+=1;
											}
											*glat_a = gga_lat;

											if (*ki == ',')
											{
												*ki = UART_recieve();

												if (*ki == 'N' || *ki == 'S'){
													gga_dlat[0]=*ki;
													*ki = UART_recieve();
													*dg_lat = gga_dlat;

													if (*ki == ','){
														t=0;
														*ki = UART_recieve();
														while(*ki !=','){
															gga_lng[t] = *ki;
															*ki = UART_recieve();
															t+=1;
														}
														*glng_a = gga_lng;

														if (*ki == ',')
														{
															*ki = UART_recieve();
															if (*ki == 'E' || *ki == 'W')
															{
																gga_dlng[0]=*ki;
																*ki = UART_recieve();
																*dg_lng = gga_dlng;
																if (*ki == ',')
																{
																	*ki = UART_recieve();
																	if (*ki != ',')
																	{

																		fix_ty[0] = *ki;
																		*ki = UART_recieve();
																		*gfix_ty = fix_ty;
																		if (*ki == ',')
																		{
																			t=0;
																			*ki = UART_recieve();
																			while (*ki != ',')
																			{
																				nsat[t] = *ki;
																				*ki = UART_recieve();
																				t+=1;
																			}
																				if (*ki == ',')
																				{
																					t=0;
																					*ki = UART_recieve();

																					while (*ki != ',')
																					{
																						hdop[t] = *ki;
																						*ki=UART_recieve();
																						t+=1;
																					}
																					*ghdop = hdop;
																					if (*ki == ',')
																					{
																						t=0;
																						*ki = UART_recieve();

																						while (*ki != ',')
																						{
																							gga_alt[t] = *ki;
																							*ki=UART_recieve();
																							t+=1;
																						}
																						*galt = gga_alt;
																						if (*ki == ',')
																						{
																							*ki = UART_recieve();
																							if (*ki == 'M')
																							{
																								gga_dalt[0] = *ki;
																								*ki = UART_recieve();
																								*gdalt = gga_dalt;
																							}
																						}

																					}

																				}
																		}
																	}
																}

															}
														}
													}
												}
											}
										}
									}
								}
							}
						}
					}
				}
			}
		}
	}
}


double initial_bearing(double lat_a,double lat_b,double lon_a,double lon_b){
	double delta_lng;
	double lat1=ConvertDegtoRad(lat_a);
	double lat2=ConvertDegtoRad(lat_b);
	double lon1=ConvertDegtoRad(lon_a);
	double lon2=ConvertDegtoRad(lon_b);

	delta_lng = lon2-lon1;
	float y = sin(delta_lng) * cos(lat2);
	float x = cos(lat1)*sin(lat2) -sin(lat1)*cos(lat2)*cos(delta_lng);
	double bearing= ConvertRadtoDeg(atan2(y,x));

	double final;
	if (bearing>0){
		final=bearing;
		}else {
		final=360+bearing;
	}

	return final;
}
/*
double final_bearing(double lat_a,double lat_b,double lon_a,double lon_b){
	double final;
	double inti = initial_bearing(lat_a,lat_b,lon_a,lon_b);

	if(inti<180){
		final =(inti+180);
		}else{
		final= (inti-180);
	}
	return final;
}*/


double conv_rhumb(double pts[],double pts_p[]){
	float delta_lng,mean_lat;
	delta_lng = pts[1]-pts_p[1];
	mean_lat = (pts[0] + pts_p[0])/2;
	double rad = ConvertDegtoRad((double)mean_lat);
	double con =(double)(delta_lng * sin(rad));
	return con;
}
double distance(double lat1,double lat2,double lon1,double lon2){


	double  dlon = ConvertDegtoRad(lon2 - lon1) ;
	double dlat = ConvertDegtoRad(lat2 - lat1) ;
	double deg_lat1 = ConvertDegtoRad(lat1);
	double deg_lat2 = ConvertDegtoRad(lat2);
	double a=sin(dlat/2)*sin(dlat/2) + (cos(deg_lat1) * cos(deg_lat2))* (sin(dlon/2)*sin(dlon/2));
	double c=2 * atan2(sqrt(a),sqrt(1-a));
	double R= 6371000.0 ;
	double dist=c*R;
	return dist;
}

double cross_track_error(double cur_n,double bear_way, double curr_bear){
	double R= 6371000.0;
	double ang_disc = cur_n / R;
	double delta_bearing =ConvertDegtoRad( curr_bear - bear_way);
	double values=sin(ang_disc)*sin(delta_bearing);
	double cte= asin(values)*R;
	return cte;
}

double cross_track_error_a(double cur_n,double bear_way, double curr_bear){
	double delta_bearing =ConvertDegtoRad( curr_bear - bear_way);
	double perd = cur_n*sin(delta_bearing);
	return perd;
}


float along_track_error(double curk,double cte_n){
	float alg = sqrt((curk*curk) - (cte_n*cte_n));
	return alg;
}



void id_vtg(char** course, char** refn, char** coursem, char** refm, char** speedn, char** unitn, char** speedk, char** unitk, char** modevtg) {
	uint8_t t = 0;
	char *ki = '\0';
	char vtg_course[10] = {'\0'};      
	char vtg_ref[2] = {'\0'};          
	char vtg_coursem[10] = {'\0'};    
	char vtg_refm[2] = {'\0'};        
	char vtg_speedn[10] = {'\0'};     
	char vtg_unitn[2] = {'\0'};        
	char vtg_speedk[10] = {'\0'};     
	char vtg_unitk[2] = {'\0'};       
	char vtg_mode[2] = {'\0'};         // Mode (A, D, E)
    char temp[10] = {'\0'};

	while (UART_Available() > 0) {
		
		*ki = UART_recieve();
		if (*ki == '$') {
			*ki = UART_recieve();
			if (*ki == 'G') {
				*ki = UART_recieve();
				if (*ki == 'N') {
					*ki = UART_recieve();
					if (*ki == 'V') {
						*ki = UART_recieve();
						if (*ki == 'T') {
							*ki = UART_recieve();
							if (*ki == 'G') {
								//Write_String("VTG recieved \n");
								//,T,,M,0.073,N,0.135,K,A
								*ki = UART_recieve();
								if (*ki == ',') {
									// Read Course (True)
									*ki = UART_recieve();
									t = 0;
									while (*ki != ',') {
										vtg_course[t] = *ki;
										
										*ki = UART_recieve();
										t++;
									}
									*course = vtg_course;
									if(*ki==','){
										*ki = UART_recieve();
										if(*ki=='T'){
											vtg_ref[0]==*ki;
											*refn=vtg_ref;
											*ki = UART_recieve();
											
											if(*ki==','){
												*ki = UART_recieve();
												t=0;
												while(*ki!=','){
													vtg_coursem[t]=*ki;
													*ki = UART_recieve();
													t++;
												}
												
												*coursem=vtg_coursem;
												if(*ki==','){
													*ki = UART_recieve();
													if(*ki=='M'){
														vtg_refm[0]=*ki;
														*refm=vtg_refm;
														
														*ki = UART_recieve();
														if(*ki==','){
														*ki = UART_recieve();
														t=0;
														while(*ki!=','){
														vtg_speedn[t]=*ki;
														*ki = UART_recieve();
														t++;
														}
														*speedn=vtg_speedn;
														if(*ki==','){
															*ki = UART_recieve();
															if(*ki=='N'){
																vtg_unitn[0]=*ki;
																*unitn=vtg_unitn;
																*ki = UART_recieve();
																if(*ki==','){
																	*ki = UART_recieve();
																	t=0;
																	
																	while(*ki!=','){
																		vtg_speedk[t]=*ki;
																		*ki = UART_recieve();
																		t++;
																	}
																	*speedk=vtg_speedk;
																	if(*ki==','){
																		*ki = UART_recieve();
																		if(*ki=='K'){
																			vtg_unitk[0]=*ki;
																			*unitk=vtg_unitk;
																			*ki = UART_recieve();
																			if(*ki==','){
		                                                                      *ki = UART_recieve();
																			  if(*ki=='A'||*ki=='D'||*ki=='E'){
																				  vtg_mode[0]=*ki;
																				  *modevtg=vtg_mode;
																				  
																			  }
																			}
																		}
																	}
																	
																}
															}
														}
													}
												}
											}
										}
		
									}
										
                                   
								}
							}
						}
					}
				}
			}
		}
	}
}
	}
