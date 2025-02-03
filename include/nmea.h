/*
 * nmea.h
 *
 * Created: 24-08-2024 13:54:00
 *  Author: sumit.chatterjee
 */ 


#ifndef NMEA_H_
#define NMEA_H_


double lng_filt(float kef);
double lat_filt(float def);
void id_rmc(char** lat_a,char** lng_a, char** dr_lat,char** dr_lng,char** spd,char** ctim,char** cdat,char **cors,char **magh);
void id_gga(char** glat_a,char** glng_a, char** dg_lat,char** dg_lng,char** ghdop,char** gtim,char** galt,char** gdalt,char** fix_ty);
void id_vtg(char** course,char** ref, char** coursem,char** refm,char** speedn,char** unitn,char** speedk,char** unitk,char** modevtg);
void convert_time_to_UTC(unsigned long int UTC_Time);
double ConvertRadtoDeg(double radians);
double ConvertDegtoRad(double degree);
double initial_bearing(double lat_a,double lat_b,double lon_a,double lon_b);
double final_bearing(double lat_a,double lat_b,double lon_a,double lon_b);
double cross_track_error(double cur_n,double bear_way, double curr_bear);
float along_track_error(double cur_na,double cte_k);
double cross_track_error_a(double cur_n,double bear_way, double curr_bear);
double distance(double lat1,double lat2,double lon1,double lon2);



#endif /* NMEA_H_ */