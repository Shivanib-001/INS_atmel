/*
 * rtc.h
 *
 * Created: 21-08-2024 12:56:49
 *  Author: sumit.chatterjee
 */ 


#ifndef RTC_H_
#define RTC_H_
#include "i2c.h"
#include <avr/sfr_defs.h>
#include <util/twi.h>
#include <compat/twi.h>

enum options {SECOND, MINUTE, HOUR, DAY_OF_WEEK, DATE, MONTH, YEAR, CONTROL, RAM, TIME, SNAPSHOT, ALL};
enum square_wave {WAVE_OFF, WAVE_1, WAVE_2, WAVE_3, WAVE_4};
enum ds1307_registers {
	DS1307_REGISTER_SECONDS,
	DS1307_REGISTER_MINUTES,
	DS1307_REGISTER_HOURS,
	DS1307_REGISTER_DAY_OF_WEEK,
	DS1307_REGISTER_DATE,
	DS1307_REGISTER_MONTH,
	DS1307_REGISTER_YEAR,
    DS1307_REGISTER_CONTROL
};


enum ds3213_registers {
	DS3213_REGISTER_SECONDS,
	DS3213_REGISTER_MINUTES,
	DS3213_REGISTER_HOURS,
	DS3213_REGISTER_DAY_OF_WEEK,
	DS3213_REGISTER_DATE,
	DS3213_REGISTER_MONTH,
	DS3213_REGISTER_YEAR,
	DS3213_REGISTER_CONTROL
};

#define DS1307_I2C_ADDRESS                    0X68

#define CLOCK_RUN                             0X01
#define CLOCK_HALT                            0X00
#define FORCE_RESET                           0X00
#define NO_FORCE_RESET                        0X01
#define DS1307_IS_RUNNING                     0X01
#define DS1307_IS_STOPPED                     0X00
#define OPERATION_DONE                        0X01
#define OPERATION_FAILED                      0X00
#define DS1307_NOT_INITIALIZED                0X00
#define DS1307_INITIALIZED                    0X2C
#define OCCUPIED                              0X01
#define NOT_OCCUPIED                          0X00

#define DS1307_REGISTER_INIT_STATUS           0X08
#define DS1307_REGISTER_SNAP0_VACANCY         0X09
#define DS1307_RAM_START                      0X08
#define DS1307_RAM_END                        0X3F
#define DS1307_BIT_SETTING_CH                 0X07
#define DS1307_BIT_SETTING_RS0                0X00
#define DS1307_BIT_SETTING_RS1                0X01
#define DS1307_BIT_SETTING_SQWE               0X04
#define DS1307_BIT_SETTING_OUT                0X07
#define DS1307_BIT_SETTING_AMPM               0X06
#define DS1307_TIMEKEEPER_REGISTERS_START     0X00
#define DS1307_TIMEKEEPER_REGISTERS_END       0X07
#define DS1307_REGISTER_SECONDS_DEFAULT       0X00
#define DS1307_REGISTER_MINUTES_DEFAULT       0X00
#define DS1307_REGISTER_HOURS_DEFAULT         0X00
#define DS1307_REGISTER_DAY_OF_WEEK_DEFAULT   0X01
#define DS1307_REGISTER_DATE_DEFAULT          0X01
#define DS1307_REGISTER_MONTH_DEFAULT         0X01
#define DS1307_REGISTER_YEAR_DEFAULT          0X00
#define DS1307_REGISTER_CONTROL_DEFAULT       0X00
#define DS1307_RAM_BLOCK_DEFAULT              0x00
#define DS1307_SNAP0_ADDRESS                  0X39


#define DS3231_REGISTER_CONTROL 0xe
#define DS3231_REGISTER_CONTROL_STATUS 0xf
#define DS3231_REGISTER_SECONDS 0x0
#define DS3231_REGISTER_MINUTES 0x1
#define DS3231_REGISTER_HOURS 0x2
#define DS3231_REGISTER_DAY 0x3
#define DS3231_REGISTER_DATE 0x4
#define DS3231_REGISTER_MONTH_CENTURY 0x5
#define DS3231_REGISTER_YEAR 0x6

void rtc_init();
void rtc_setTime(unsigned char h, unsigned char m, unsigned char s);
void Rtc_setTime(uint8_t hs, uint8_t ms, uint8_t ss);
void Rtc_getTime(unsigned char**gsi, unsigned char **gmi,unsigned char **ghi);
void rtc_getTime(unsigned char **gs, unsigned char **gm, unsigned char **gh);
uint8_t ds3231GetSecond(void);
void ds3231_setRegisterPointer(uint8_t reg);
uint8_t ds3231_getRegisterValue(uint8_t reg);
uint8_t ds3213_write(uint8_t reg,uint8_t values);



#endif /* RTC_H_ */