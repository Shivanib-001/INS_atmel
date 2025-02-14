/*
 * bno055.h
 *
 * Created: 22-01-2025 14:30:17
 *  Author: kishan.shivhare
 */ 


#ifndef BNO055_H_
#define BNO055_H_

#include <avr/io.h>
#include "uart.h"
#include "i2c.h"
#include <avr/sfr_defs.h>
#include <util/twi.h>
#include <compat/twi.h>
#include <stdio.h>
#include <string.h>


#define BNO055_ADDRESS          0x28
#define BNO055_CHIP_ID          0x00    // should be 0xA0
#define BNO055_ACC_ID           0x01    // should be 0xFB
#define BNO055_MAG_ID           0x02    // should be 0x32
#define BNO055_GYRO_ID          0x03    // should be 0x0F
#define BNO055_SW_REV_ID_LSB    0x04
#define BNO055_SW_REV_ID_MSB    0x05
#define BNO055_BL_REV_ID        0x06
#define BNO055_PAGE_ID          0x07
#define BNO055_ACC_DATA_X_LSB   0x08
#define BNO055_ACC_DATA_X_MSB   0x09
#define BNO055_ACC_DATA_Y_LSB   0x0A
#define BNO055_ACC_DATA_Y_MSB   0x0B
#define BNO055_ACC_DATA_Z_LSB   0x0C
#define BNO055_ACC_DATA_Z_MSB   0x0D
#define BNO055_MAG_DATA_X_LSB   0x0E
#define BNO055_MAG_DATA_X_MSB   0x0F
#define BNO055_MAG_DATA_Y_LSB   0x10
#define BNO055_MAG_DATA_Y_MSB   0x11
#define BNO055_MAG_DATA_Z_LSB   0x12
#define BNO055_MAG_DATA_Z_MSB   0x13
#define BNO055_GYR_DATA_X_LSB   0x14
#define BNO055_GYR_DATA_X_MSB   0x15
#define BNO055_GYR_DATA_Y_LSB   0x16
#define BNO055_GYR_DATA_Y_MSB   0x17
#define BNO055_GYR_DATA_Z_LSB   0x18
#define BNO055_GYR_DATA_Z_MSB   0x19
#define BNO055_EUL_HEADING_LSB  0x1A
#define BNO055_EUL_HEADING_MSB  0x1B
#define BNO055_EUL_ROLL_LSB     0x1C
#define BNO055_EUL_ROLL_MSB     0x1D
#define BNO055_EUL_PITCH_LSB    0x1E
#define BNO055_EUL_PITCH_MSB    0x1F
#define BNO055_QUA_DATA_W_LSB   0x20
#define BNO055_QUA_DATA_W_MSB   0x21
#define BNO055_QUA_DATA_X_LSB   0x22
#define BNO055_QUA_DATA_X_MSB   0x23
#define BNO055_QUA_DATA_Y_LSB   0x24
#define BNO055_QUA_DATA_Y_MSB   0x25
#define BNO055_QUA_DATA_Z_LSB   0x26
#define BNO055_QUA_DATA_Z_MSB   0x27
#define BNO055_LIA_DATA_X_LSB   0x28
#define BNO055_LIA_DATA_X_MSB   0x29
#define BNO055_LIA_DATA_Y_LSB   0x2A
#define BNO055_LIA_DATA_Y_MSB   0x2B
#define BNO055_LIA_DATA_Z_LSB   0x2C
#define BNO055_LIA_DATA_Z_MSB   0x2D
#define BNO055_GRV_DATA_X_LSB   0x2E
#define BNO055_GRV_DATA_X_MSB   0x2F
#define BNO055_GRV_DATA_Y_LSB   0x30
#define BNO055_GRV_DATA_Y_MSB   0x31
#define BNO055_GRV_DATA_Z_LSB   0x32
#define BNO055_GRV_DATA_Z_MSB   0x33
#define BNO055_TEMP             0x34
#define BNO055_CALIB_STAT       0x35
#define BNO055_ST_RESULT        0x36
#define BNO055_INT_STATUS       0x37
#define BNO055_SYS_CLK_STATUS   0x38
#define BNO055_SYS_STATUS       0x39
#define BNO055_SYS_ERR          0x3A
#define BNO055_UNIT_SEL         0x3B
#define BNO055_OPR_MODE         0x3D
#define BNO055_PWR_MODE         0x3E
#define BNO055_SYS_TRIGGER      0x3F
#define BNO055_TEMP_SOURCE      0x40
#define BNO055_AXIS_MAP_CONFIG  0x41
#define BNO055_AXIS_MAP_SIGN    0x42
#define BNO055_ACC_OFFSET_X_LSB 0x55
#define BNO055_ACC_OFFSET_X_MSB 0x56
#define BNO055_ACC_OFFSET_Y_LSB 0x57
#define BNO055_ACC_OFFSET_Y_MSB 0x58
#define BNO055_ACC_OFFSET_Z_LSB 0x59
#define BNO055_ACC_OFFSET_Z_MSB 0x5A
#define BNO055_MAG_OFFSET_X_LSB 0x5B
#define BNO055_MAG_OFFSET_X_MSB 0x5C
#define BNO055_MAG_OFFSET_Y_LSB 0x5D
#define BNO055_MAG_OFFSET_Y_MSB 0x5E
#define BNO055_MAG_OFFSET_Z_LSB 0x5F
#define BNO055_MAG_OFFSET_Z_MSB 0x60
#define BNO055_GYR_OFFSET_X_LSB 0x61
#define BNO055_GYR_OFFSET_X_MSB 0x62
#define BNO055_GYR_OFFSET_Y_LSB 0x63
#define BNO055_GYR_OFFSET_Y_MSB 0x64
#define BNO055_GYR_OFFSET_Z_LSB 0x65
#define BNO055_GYR_OFFSET_Z_MSB 0x66
#define BNO055_ACC_RADIUS_LSB   0x67
#define BNO055_ACC_RADIUS_MSB   0x68
#define BNO055_MAG_RADIUS_LSB   0x69
#define BNO055_MAG_RADIUS_MSB   0x6A
//
// BNO055 Page 1
#define BNO055_PAGE_ID          0x07
#define BNO055_ACC_CONFIG       0x08
#define BNO055_MAG_CONFIG       0x09
#define BNO055_GYRO_CONFIG_0    0x0A
#define BNO055_GYRO_CONFIG_1    0x0B
#define BNO055_ACC_SLEEP_CONFIG 0x0C
#define BNO055_GYR_SLEEP_CONFIG 0x0D
#define BNO055_INT_MSK          0x0F
#define BNO055_INT_EN           0x10
#define BNO055_ACC_AM_THRES     0x11
#define BNO055_ACC_INT_SETTINGS 0x12
#define BNO055_ACC_HG_DURATION  0x13
#define BNO055_ACC_HG_THRESH    0x14
#define BNO055_ACC_NM_THRESH    0x15
#define BNO055_ACC_NM_SET       0x16
#define BNO055_GYR_INT_SETTINGS 0x17
#define BNO055_GYR_HR_X_SET     0x18
#define BNO055_GYR_DUR_X        0x19
#define BNO055_GYR_HR_Y_SET     0x1A
#define BNO055_GYR_DUR_Y        0x1B
#define BNO055_GYR_HR_Z_SET     0x1C
#define BNO055_GYR_DUR_Z        0x1D
#define BNO055_GYR_AM_THRESH    0x1E
#define BNO055_GYR_AM_SET       0x1F


// Using the BNO055_BMP280 breakout board/Teensy 3.1 Add-On Shield, ADO is set to 1 by default
//#define ADO 1
//#if ADO
//#define BNO055_ADDRESS 0x29   //  Device address of BNO055 when ADO = 1
//#else
//#define BNO055_ADDRESS 0x28   //  Device address of BNO055 when ADO = 0
//#endif
//
//#define SerialDebug true      // set to true to get Serial output for debugging

// Set initial input parameters
enum Ascale {  // ACC Full Scale
	AFS_2G = 0,
	AFS_4G,
	AFS_8G,
	AFS_18G
};

enum Abw { // ACC Bandwidth
	ABW_7_81Hz = 0,
	ABW_15_63Hz,
	ABW_31_25Hz,
	ABW_62_5Hz,
	ABW_125Hz,
	ABW_250Hz,
	ABW_500Hz,
	ABW_1000Hz,    //0x07
};

enum APwrMode { // ACC Pwr Mode
	NormalA = 0,
	SuspendA,
	LowPower1A,
	StandbyA,
	LowPower2A,
	DeepSuspendA
};

enum Gscale {  // gyro full scale
	GFS_2000DPS = 0,
	GFS_1000DPS,
	GFS_500DPS,
	GFS_250DPS,
	GFS_125DPS    // 0x04
};

enum GPwrMode { // GYR Pwr Mode
	NormalG = 0,
	FastPowerUpG,
	DeepSuspendedG,
	SuspendG,
	AdvancedPowerSaveG
};

enum Gbw { // gyro bandwidth
	GBW_523Hz = 0,
	GBW_230Hz,
	GBW_116Hz,
	GBW_47Hz,
	GBW_23Hz,
	GBW_12Hz,
	GBW_64Hz,
	GBW_32Hz
};

enum OPRMode {  // BNO-55 operation modes
	CONFIGMODE = 0x00,
	// Sensor Mode
	ACCONLY,
	MAGONLY,
	GYROONLY,
	ACCMAG,
	ACCGYRO,
	MAGGYRO,
	AMG,            // 0x07
	// Fusion Mode
	IMU,
	COMPASS,
	M4G,
	NDOF_FMC_OFF,
	NDOF            // 0x0C
};

enum PWRMode {
	Normalpwr = 0,
	Lowpower,
	Suspendpwr
};

enum Modr {         // magnetometer output data rate
	MODR_2Hz = 0,
	MODR_6Hz,
	MODR_8Hz,
	MODR_10Hz,
	MODR_15Hz,
	MODR_20Hz,
	MODR_25Hz,
	MODR_30Hz
};

enum MOpMode { // MAG Op Mode
	LowPower = 0,
	Regular,
	EnhancedRegular,
	HighAccuracy
};

enum MPwrMode { // MAG power mode
	Normal = 0,
	Sleep,
	Suspend,
	ForceMode
};




//void BNO_Scan();
//void BNO_POST();
//void BNO_Calib();
//void BNO_init();
//void SetMode(OPRMode_t mode);
//void GetMode();
//void SetPowerMode(PWRMode_t mode_p);
//void i2c_writeData(uint8_t regaddr,uint8_t txsubaddr, uint8_t data);
//void i2c_readData(uint8_t addr , uint8_t subaddr,uint8_t * buf, uint8_t buf_size);
//void readAccelData(int16_t * destination);
//void readGyroData(int16_t * destination);
//void readQuatData(int16_t * destination);
//void readLIAData(int16_t * destination);
//void readGRVData(int16_t * destination);
//void readMagData(int16_t * destination);
//void readEulData(int16_t * destination);
//void BNO_SetAccOffset(double * dest_a);
//void BNO_SetGyroOffset(float * dest_a);
//void BNO_SetMagOffset(float * dest_a);
//

void readAccelData(int16_t * destination);
void readGyroData(int16_t * destination);
int8_t readGyroTempData();
void readMagData(int16_t * destination);
void readQuatData(int16_t * destination);
void readEulData(float * destination);
void readLIAData(int16_t * destination);
void readGRVData(int16_t * destination);
void initBNO055();
void WHO_AM_I();
void selfTest();
void calibration_status();



#endif /* BNO055_H_ */