/*
 * bno055.c
 *
 * Created: 22-01-2025 14:33:10
 *  Author: kishan.shivhare
 */ 

//===================================================================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================
#include "bno055.h"

uint8_t GPwrMode = NormalG;    // Gyro power mode
uint8_t Gscale = GFS_250DPS;  // Gyro full scale
//uint8_t Godr = GODR_250Hz;    // Gyro sample rate
uint8_t Gbw = GBW_23Hz;       // Gyro bandwidth
//
uint8_t Ascale = AFS_2G;      // Accel full scale
//uint8_t Aodr = AODR_250Hz;    // Accel sample rate
uint8_t APwrMode = NormalA;    // Accel power mode
uint8_t Abw = ABW_31_25Hz;    // Accel bandwidth, accel sample rate divided by ABW_divx
//
//uint8_t Mscale = MFS_4Gauss;  // Select magnetometer full-scale resolution
uint8_t MOpMode = Regular;    // Select magnetometer perfomance mode
uint8_t MPwrMode = Normal;    // Select magnetometer power mode
uint8_t Modr = MODR_10Hz;     // Select magnetometer ODR when in BNO055 bypass mode

uint8_t PWRMode = Normalpwr;    // Select BNO055 power mode
uint8_t OPRMode = NDOF;       // specify operation mode for sensors
uint8_t status;               // BNO055 data status register
float aRes, gRes, mRes;       // scale resolutions per LSB for the sensors
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}, magBias[3] = {0, 0, 0};  // Bias corrections for gyro, accelerometer, and magnetometer

void readAccelData(int16_t * destination)
{
	uint8_t rawData[6];  // x/y/z accel register data stored here
	readBytes(BNO055_ADDRESS, BNO055_ACC_DATA_X_LSB, 6, &rawData[0]);  // Read the six raw data registers into data array
	destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;      // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;
	destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
}


void readGyroData(int16_t * destination)
{
	uint8_t rawData[6];  // x/y/z gyro register data stored here
	readBytes(BNO055_ADDRESS, BNO055_GYR_DATA_X_LSB, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
	destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;       // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;
	destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
}

int8_t readGyroTempData()
{
	return readByte(BNO055_ADDRESS, BNO055_TEMP);  // Read the two raw data registers sequentially into data array
}

void readMagData(int16_t * destination)
{
	uint8_t rawData[6];  // x/y/z gyro register data stored here
	readBytes(BNO055_ADDRESS, BNO055_MAG_DATA_X_LSB, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
	destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;       // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;
	destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
}

void readQuatData(int16_t * destination)
{
	uint8_t rawData[8];  // x/y/z gyro register data stored here
	readBytes(BNO055_ADDRESS, BNO055_QUA_DATA_W_LSB, 8, &rawData[0]);  // Read the six raw data registers sequentially into data array
	destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;       // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;
	destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
	destination[3] = ((int16_t)rawData[7] << 8) | rawData[6] ;
}

void readEulData(float * destination)
{
	uint8_t rawData[6];  // x/y/z gyro register data stored here
	readBytes(BNO055_ADDRESS, BNO055_EUL_HEADING_LSB, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
	destination[0] = (float)(((int16_t)rawData[1] << 8) | rawData[0])/16. ;       // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = (float)(((int16_t)rawData[3] << 8) | rawData[2])/16. ;
	destination[2] = (float)(((int16_t)rawData[5] << 8) | rawData[4])/16. ;
}

void readLIAData(int16_t * destination)
{
	uint8_t rawData[6];  // x/y/z gyro register data stored here
	readBytes(BNO055_ADDRESS, BNO055_LIA_DATA_X_LSB, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
	destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;       // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;
	destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
}

void readGRVData(int16_t * destination)
{
	uint8_t rawData[6];  // x/y/z gyro register data stored here
	readBytes(BNO055_ADDRESS, BNO055_GRV_DATA_X_LSB, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
	destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;       // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;
	destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
}

void initBNO055() {
	// Select BNO055 config mode
	writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, CONFIGMODE );
	_delay_ms(25);
	// Select page 1 to configure sensors
	writeByte(BNO055_ADDRESS, BNO055_PAGE_ID, 0x01);
	// Configure ACC
	writeByte(BNO055_ADDRESS, BNO055_ACC_CONFIG, APwrMode << 5 | Abw << 2 | Ascale );
	// Configure GYR
	writeByte(BNO055_ADDRESS, BNO055_GYRO_CONFIG_0, Gbw << 3 | Gscale );
	writeByte(BNO055_ADDRESS, BNO055_GYRO_CONFIG_1, GPwrMode);
	// Configure MAG
	writeByte(BNO055_ADDRESS, BNO055_MAG_CONFIG, MPwrMode << 5 | MOpMode << 3 | Modr );
	
	// Select page 0 to read sensors
	writeByte(BNO055_ADDRESS, BNO055_PAGE_ID, 0x00);

	// Select BNO055 gyro temperature source
	writeByte(BNO055_ADDRESS, BNO055_TEMP_SOURCE, 0x01 );

	// Select BNO055 sensor units (temperature in degrees C, rate in dps, accel in mg)
	writeByte(BNO055_ADDRESS, BNO055_UNIT_SEL, 0x01 );
	
	// Select BNO055 system power mode
	writeByte(BNO055_ADDRESS, BNO055_PWR_MODE, PWRMode );
	
	// Select BNO055 system operation mode
	writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, OPRMode );
	_delay_ms(25);
}

void accelgyroCalBNO055(float * dest1, float * dest2)
{
	uint8_t data[6]; // data array to hold accelerometer and gyro x, y, z, data
	uint16_t ii = 0, sample_count = 0;
	int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
	
	WriteStringn("Accel/Gyro Calibration: Put device on a level surface and keep motionless! Wait......");
	_delay_ms(4000);
	
	// Select page 0 to read sensors
	writeByte(BNO055_ADDRESS, BNO055_PAGE_ID, 0x00);
	// Select BNO055 system operation mode as AMG for calibration
	writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, CONFIGMODE );
	_delay_ms(25);
	writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, AMG);
	
	// In NDF fusion mode, accel full scale is at +/- 4g, ODR is 62.5 Hz, set it the same here
	writeByte(BNO055_ADDRESS, BNO055_ACC_CONFIG, APwrMode << 5 | Abw << 2 | AFS_4G );
	sample_count = 256;
	for(ii = 0; ii < sample_count; ii++) {
		int16_t accel_temp[3] = {0, 0, 0};
		readBytes(BNO055_ADDRESS, BNO055_ACC_DATA_X_LSB, 6, &data[0]);  // Read the six raw data registers into data array
		accel_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]) ; // Form signed 16-bit integer for each sample in FIFO
		accel_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]) ;
		accel_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]) ;
		accel_bias[0]  += (int32_t) accel_temp[0];
		accel_bias[1]  += (int32_t) accel_temp[1];
		accel_bias[2]  += (int32_t) accel_temp[2];
		_delay_ms(20);  // at 62.5 Hz ODR, new accel data is available every 16 ms
	}
	accel_bias[0]  /= (int32_t) sample_count;  // get average accel bias in mg
	accel_bias[1]  /= (int32_t) sample_count;
	accel_bias[2]  /= (int32_t) sample_count;
	
	if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) 1000;}  // Remove gravity from the z-axis accelerometer bias calculation
	else {accel_bias[2] += (int32_t) 1000;}

	dest1[0] = (float) accel_bias[0];  // save accel biases in mg for use in main program
	dest1[1] = (float) accel_bias[1];  // accel data is 1 LSB/mg
	dest1[2] = (float) accel_bias[2];

	// In NDF fusion mode, gyro full scale is at +/- 2000 dps, ODR is 32 Hz
	writeByte(BNO055_ADDRESS, BNO055_GYRO_CONFIG_0, Gbw << 3 | GFS_2000DPS );
	writeByte(BNO055_ADDRESS, BNO055_GYRO_CONFIG_1, GPwrMode);for(ii = 0; ii < sample_count; ii++) {
		int16_t gyro_temp[3] = {0, 0, 0};
		readBytes(BNO055_ADDRESS, BNO055_GYR_DATA_X_LSB, 6, &data[0]);  // Read the six raw data registers into data array
		gyro_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]) ;  // Form signed 16-bit integer for each sample in FIFO
		gyro_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]) ;
		gyro_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]) ;
		gyro_bias[0]  += (int32_t) gyro_temp[0];
		gyro_bias[1]  += (int32_t) gyro_temp[1];
		gyro_bias[2]  += (int32_t) gyro_temp[2];
		_delay_ms(35);  // at 32 Hz ODR, new gyro data available every 31 ms
	}
	gyro_bias[0]  /= (int32_t) sample_count;  // get average gyro bias in counts
	gyro_bias[1]  /= (int32_t) sample_count;
	gyro_bias[2]  /= (int32_t) sample_count;
	
	dest2[0] = (float) gyro_bias[0]/16.;  // save gyro biases in dps for use in main program
	dest2[1] = (float) gyro_bias[1]/16.;  // gyro data is 16 LSB/dps
	dest2[2] = (float) gyro_bias[2]/16.;

	// Return to config mode to write accelerometer biases in offset register
	// This offset register is only used while in fusion mode when accelerometer full-scale is +/- 4g
	writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, CONFIGMODE );
	_delay_ms(25);
	
	//write biases to accelerometer offset registers ad 16 LSB/dps
	writeByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_X_LSB, (int16_t)accel_bias[0] & 0xFF);
	writeByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_X_MSB, ((int16_t)accel_bias[0] >> 8) & 0xFF);
	writeByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Y_LSB, (int16_t)accel_bias[1] & 0xFF);
	writeByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Y_MSB, ((int16_t)accel_bias[1] >> 8) & 0xFF);
	writeByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Z_LSB, (int16_t)accel_bias[2] & 0xFF);
	writeByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Z_MSB, ((int16_t)accel_bias[2] >> 8) & 0xFF);
	
	// Check that offsets were properly written to offset registers
	//  Serial.println("Average accelerometer bias = ");
	//  Serial.println((int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_X_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_X_LSB)));
	//  Serial.println((int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Y_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Y_LSB)));
	//  Serial.println((int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Z_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Z_LSB)));

	//write biases to gyro offset registers
	writeByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_X_LSB, (int16_t)gyro_bias[0] & 0xFF);
	writeByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_X_MSB, ((int16_t)gyro_bias[0] >> 8) & 0xFF);
	writeByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Y_LSB, (int16_t)gyro_bias[1] & 0xFF);
	writeByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Y_MSB, ((int16_t)gyro_bias[1] >> 8) & 0xFF);
	writeByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Z_LSB, (int16_t)gyro_bias[2] & 0xFF);
	writeByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Z_MSB, ((int16_t)gyro_bias[2] >> 8) & 0xFF);
	
	// Select BNO055 system operation mode
	writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, OPRMode );

	// Check that offsets were properly written to offset registers
	//  Serial.println("Average gyro bias = ");
	//  Serial.println((int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_X_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_X_LSB)));
	//  Serial.println((int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Y_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Y_LSB)));
	//  Serial.println((int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Z_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Z_LSB)));

	WriteStringn("Accel/Gyro Calibration done!");
}

void magCalBNO055(float * dest1)
{
	uint8_t data[6]; // data array to hold mag x, y, z, data
	uint16_t ii = 0, sample_count = 0;
	int32_t mag_bias[3] = {0, 0, 0};
	int16_t mag_max[3] = {0, 0, 0}, mag_min[3] = {0, 0, 0};
	
	WriteStringn("Mag Calibration: Wave device in a figure eight until done!");
	_delay_ms(4000);
	
	// Select page 0 to read sensors
	writeByte(BNO055_ADDRESS, BNO055_PAGE_ID, 0x00);
	// Select BNO055 system operation mode as NDOF for calibration
	writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, CONFIGMODE );
	_delay_ms(25);
	writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, AMG );

	// In NDF fusion mode, mag data is in 16 LSB/microTesla, ODR is 20 Hz in forced mode
	sample_count = 256;
	for(ii = 0; ii < sample_count; ii++) {
		int16_t mag_temp[3] = {0, 0, 0};
		readBytes(BNO055_ADDRESS, BNO055_MAG_DATA_X_LSB, 6, &data[0]);  // Read the six raw data registers into data array
		mag_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]) ;   // Form signed 16-bit integer for each sample in FIFO
		mag_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]) ;
		mag_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]) ;
		for (int jj = 0; jj < 3; jj++) {
			if (ii == 0) {
				mag_max[jj] = mag_temp[jj]; // Offsets may be large enough that mag_temp[i] may not be bipolar!
				mag_min[jj] = mag_temp[jj]; // This prevents max or min being pinned to 0 if the values are unipolar...
				} else {
				if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
				if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
			}
		}
		_delay_ms(105);  // at 10 Hz ODR, new mag data is available every 100 ms
	}

	//   Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
	//   Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
	//   Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

	mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
	mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
	mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts
	
	dest1[0] = (float) mag_bias[0] / 1.6;  // save mag biases in mG for use in main program
	dest1[1] = (float) mag_bias[1] / 1.6;  // mag data is 1.6 LSB/mg
	dest1[2] = (float) mag_bias[2] / 1.6;

	// Return to config mode to write mag biases in offset register
	// This offset register is only used while in fusion mode when magnetometer sensitivity is 16 LSB/microTesla
	writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, CONFIGMODE );
	_delay_ms(25);
	
	//write biases to accelerometer offset registers as 16 LSB/microTesla
	writeByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_X_LSB, (int16_t)mag_bias[0] & 0xFF);
	writeByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_X_MSB, ((int16_t)mag_bias[0] >> 8) & 0xFF);
	writeByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Y_LSB, (int16_t)mag_bias[1] & 0xFF);
	writeByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Y_MSB, ((int16_t)mag_bias[1] >> 8) & 0xFF);
	writeByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Z_LSB, (int16_t)mag_bias[2] & 0xFF);
	writeByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Z_MSB, ((int16_t)mag_bias[2] >> 8) & 0xFF);
	
	// Check that offsets were properly written to offset registers
	//  Serial.println("Average magnetometer bias = ");
	//  Serial.println((int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_X_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_X_LSB)));
	//  Serial.println((int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Y_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Y_LSB)));
	//  Serial.println((int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Z_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Z_LSB)));
	// Select BNO055 system operation mode
	writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, OPRMode );
	_delay_ms(25);
	
	WriteStringn("Mag Calibration done!");
}
void WHO_AM_I(){
	// Read the WHO_AM_I register, this is a good test of communication
	Write_String("BNO055 9-axis motion sensor...\n");
	uint8_t c = readByte(BNO055_ADDRESS, BNO055_CHIP_ID);  // Read WHO_AM_I register for BNO055
	Write_String("BNO055 Address = ");  UART_sendHex(BNO055_ADDRESS);Write_String("\n");
	Write_String("BNO055 WHO_AM_I = "); UART_sendHex(BNO055_CHIP_ID);Write_String("\n");
	Write_String("BNO055 "); Write_String("I AM ");UART_sendHex(c); Write_String(" I should be 0xA0");
	Write_String("\n");
	_delay_ms(1000);
	
	
	// Read the WHO_AM_I register of the accelerometer, this is a good test of communication
	uint8_t d = readByte(BNO055_ADDRESS, BNO055_ACC_ID);  // Read WHO_AM_I register for accelerometer
	Write_String("BNO055 ACC ");Write_String("I AM "); UART_sendHex(d); Write_String(" I should be 0xFB");
	Write_String("\n");
	_delay_ms(1000);
	
	// Read the WHO_AM_I register of the magnetometer, this is a good test of communication
	uint8_t e = readByte(BNO055_ADDRESS, BNO055_MAG_ID);  // Read WHO_AM_I register for magnetometer
	Write_String("BNO055 MAG "); Write_String("I AM "); UART_sendHex(e); Write_String(" I should be 0x32");
	Write_String("\n");
	_delay_ms(1000);
	
	// Read the WHO_AM_I register of the gyroscope, this is a good test of communication
	uint8_t f = readByte(BNO055_ADDRESS, BNO055_GYRO_ID);  // Read WHO_AM_I register for LIS3MDL
	Write_String("BNO055 GYRO "); Write_String("I AM "); UART_sendHex(f); Write_String(" I should be 0x0F");
	Write_String("\n");
	_delay_ms(1000);
	if (c == 0xA0) // BNO055 WHO_AM_I should always be 0xA0
	{
		WriteStringn("BNO055 is online...");
		
		// Check software revision ID
		uint8_t swlsb = readByte(BNO055_ADDRESS, BNO055_SW_REV_ID_LSB);
		uint8_t swmsb = readByte(BNO055_ADDRESS, BNO055_SW_REV_ID_MSB);
		Write_String("BNO055 SW Revision ID: "); UART_sendHex(swmsb); Write_String("."); UART_sendHex(swlsb);
		WriteStringn("Should be 03.04");
		
		// Check bootloader version
		uint8_t blid = readByte(BNO055_ADDRESS, BNO055_BL_REV_ID);
		Write_String("BNO055 bootloader Version: "); UART_sendHex(blid);
		
		// Check self-test results
		selfTest();
				_delay_ms(1000);
		
		//
		accelgyroCalBNO055(accelBias, gyroBias);
		//
		//Write_String("Average accelerometer bias (mg) = ");
		//UART_sendFloat(accelBias[0]); UART_sendFloat(accelBias[1]); UART_sendFloat(accelBias[2]);Write_String("\n");
		//Write_String("Average gyro bias (dps) = "); UART_sendFloat(gyroBias[0]); UART_sendFloat(gyroBias[1]); UART_sendFloat(gyroBias[2]);
		//Write_String("\n");
		//
		//_delay_ms(1000);
		
		//magCalBNO055(magBias);
		//
		//WriteStringn("Average magnetometer bias (mG) = "); UART_sendFloat(magBias[0]); UART_sendFloat(magBias[1]); UART_sendFloat(magBias[2]);
		//Write_String("\n");
		//_delay_ms(1000);
		
		// Check calibration status of the sensors
		calibration_status();
		
		
		
	}
	else
	{
		Write_String("Could not connect to BNO055: 0x");
		UART_sendHex(c);
		while(1) ; // Loop forever if communication doesn't happen
	}
}
void selfTest(){
	uint8_t selftest = readByte(BNO055_ADDRESS, BNO055_ST_RESULT);
	
	if(selftest & 0x01) {
		WriteStringn("accelerometer passed selftest");
		} else {
		WriteStringn("accelerometer failed selftest");
	}
	if(selftest & 0x02) {
		WriteStringn("magnetometer passed selftest");
		} else {
		WriteStringn("magnetometer failed selftest");
	}
	if(selftest & 0x04) {
		WriteStringn("gyroscope passed selftest");
		} else {
		WriteStringn("gyroscope failed selftest");
	}
	if(selftest & 0x08) {
		WriteStringn("MCU passed selftest");
		} else {
		WriteStringn("MCU failed selftest");
	}
	
}
void calibration_status(){
	uint8_t calstat = readByte(BNO055_ADDRESS, BNO055_CALIB_STAT);
	WriteStringn("Not calibrated = 0, fully calibrated = 3");
	Write_String("System calibration status "); UART_sendInt( (0xC0 & calstat) >> 6);Write_String("\n");
	Write_String("Gyro   calibration status "); UART_sendInt( (0x30 & calstat) >> 4);Write_String("\n");
	Write_String("Accel  calibration status "); UART_sendInt( (0x0C & calstat) >> 2);Write_String("\n");
	Write_String("Mag    calibration status "); UART_sendInt( (0x03 & calstat) >> 0);Write_String("\n");
}

//int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
//int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
//int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
//int16_t quatCount[4];   // Stores the 16-bit signed quaternion output
//int16_t LIACount[3];    // Stores the 16-bit signed linear acceleration output
//int16_t GRVCount[3];    // Stores the 16-bit signed gravity vector output
/*
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}, magBias[3] = {0, 0, 0};  // Bias corrections for gyro, accelerometer, and magnetometer
int16_t tempGCount, tempMCount;      // temperature raw count output of mag and gyro
float   Gtemperature, Mtemperature;  // Stores the BNO055 gyro and mag internal chip temperatures in degrees Celsius

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
//float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
//float GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense;
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy.
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
//float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
//float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

uint32_t delt_t = 0, count = 0, sumCount = 0;  // used to control display output rate
float pitch, yaw, roll;
*/

//float LIAx, LIAy, LIAz, GRVx, GRVy, GRVz;
//float deltat = 0.0f, sum = 0.0f;          // integration interval for both filter schemes
//uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
//uint32_t Now = 0;                         // used to calculate integration interval
//#define PI  3.142
//float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
//float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
//float quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
//float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

//readAccelData(accelCount);  // Read the x/y/z adc values
//// Now we'll calculate the accleration value into actual mg's
//ax = (float)accelCount[0]; // - accelBias[0];  // subtract off calculated accel bias
//ay = (float)accelCount[1]; // - accelBias[1];
//az = (float)accelCount[2]; // - accelBias[2];
//
//readGyroData(gyroCount);  // Read the x/y/z adc values
//// Calculate the gyro value into actual degrees per second
//gx = (float)gyroCount[0]/16.; // - gyroBias[0];  // subtract off calculated gyro bias
//gy = (float)gyroCount[1]/16.; // - gyroBias[1];
//gz = (float)gyroCount[2]/16.; // - gyroBias[2];
//
//readMagData(magCount);  // Read the x/y/z adc values
//// Calculate the magnetometer values in milliGauss
//mx = (float)magCount[0]/1.6; // - magBias[0];  // get actual magnetometer value in mGauss
//my = (float)magCount[1]/1.6; // - magBias[1];
//mz = (float)magCount[2]/1.6; // - magBias[2];
//
//readQuatData(quatCount);  // Read the x/y/z adc values
//// Calculate the quaternion values
//quat[0] = (float)(quatCount[0])/16384.;
//quat[1] = (float)(quatCount[1])/16384.;
//quat[2] = (float)(quatCount[2])/16384.;
//quat[3] = (float)(quatCount[3])/16384.;
//
//readLIAData(LIACount);  // Read the x/y/z adc values
//// Calculate the linear acceleration (sans gravity) values in mg
//LIAx = (float)LIACount[0];
//LIAy = (float)LIACount[1];
//LIAz = (float)LIACount[2];
//
//readGRVData(GRVCount);  // Read the x/y/z adc values
//// Calculate the linear acceleration (sans gravity) values in mg
//GRVx = (float)GRVCount[0];
//GRVy = (float)GRVCount[1];
//GRVz = (float)GRVCount[2];


//uint8_t sysstat = readByte(BNO055_ADDRESS, BNO055_SYS_STATUS); // check system statuWrite_String("System Status = 0x"); Serial.println(sysstat, HEX);
//if(sysstat == 0x05) WriteStringn("Sensor fusion algorithm running");
//if(sysstat == 0x06) WriteStringn("Sensor fusion not algorithm running");
//
//if(sysstat == 0x01) {
//uint8_t syserr = readByte(BNO055_ADDRESS, BNO055_SYS_ERR);
//if(syserr == 0x01) WriteStringn("Peripheral initialization error");
//if(syserr == 0x02) WriteStringn("System initialization error");
//if(syserr == 0x03) WriteStringn("Self test result failed");
//if(syserr == 0x04) WriteStringn("Register map value out of range");
//if(syserr == 0x05) WriteStringn("Register map address out of range");
//if(syserr == 0x06) WriteStringn("Register map write error");
//if(syserr == 0x07) WriteStringn("BNO low power mode no available for selected operation mode");
//if(syserr == 0x08) WriteStringn("Accelerometer power mode not available");
//if(syserr == 0x09) WriteStringn("Fusion algorithm configuration error");
//if(syserr == 0x0A) WriteStringn("Sensor configuration error");
//}


//Write_String("ax = "); UART_sendInt((int)ax);
//Write_String(" ay = "); UART_sendInt((int)ay);
//Write_String(" az = "); UART_sendInt((int)az); WriteStringn(" mg");
//Write_String("gx = "); UART_sendFloat( gx);
//Write_String(" gy = "); UART_sendFloat( gy);
//Write_String(" gz = "); UART_sendFloat( gz); WriteStringn(" deg/s");
//Write_String("mx = "); UART_sendInt( (int)mx );
//Write_String(" my = "); UART_sendInt( (int)my );
//Write_String(" mz = "); UART_sendInt( (int)mz ); WriteStringn(" mG");
//
//Write_String("qx = ");UART_sendFloat(q[0]);
//Write_String(" qy = ");UART_sendFloat(q[1]);
//Write_String(" qz = ");UART_sendFloat(q[2]);
//Write_String(" qw = "); UART_sendFloat(q[3]); Write_String("\n");
//Write_String("quatw = ");UART_sendFloat(quat[0]);
//Write_String(" quatx = ");UART_sendFloat(quat[1]);
//Write_String(" quaty = ");UART_sendFloat(quat[2]);
//Write_String(" quatz = "); UART_sendFloat(quat[3]); Write_String("\n");
//
//
//tempGCount = readGyroTempData();  // Read the gyro adc values
//Gtemperature = (float) tempGCount; // Gyro chip temperature in degrees Centigrade
//// Print gyro die temperature in degrees Centigrade
//Write_String("Gyro temperature is ");  UART_sendFloat(Gtemperature);  WriteStringn(" degrees C"); // Print T values to tenths of a degree C
//yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
//pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
//roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
//pitch *= 180.0f / PI;
//yaw   *= 180.0f / PI;
////   yaw   -= 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
//roll  *= 180.0f / PI;
//
//Write_String("Software Yaw, Pitch, Roll: ");
//UART_sendFloat(yaw);
//Write_String(", ");
//UART_sendFloat(pitch);
//Write_String(", ");
//UART_sendFloat(roll); Write_String("\n");
//Write_String("Hardware x, y, z linear acceleration: ");
//UART_sendFloat(LIAx);
//Write_String(", ");
//UART_sendFloat(LIAy);
//Write_String(", ");
//UART_sendFloat(LIAz); Write_String("\n");
//
//Write_String("Hardware x, y, z gravity vector: ");
//UART_sendFloat(GRVx);
//Write_String(", ");
//UART_sendFloat(GRVy);
//Write_String(", ");
//UART_sendFloat(GRVz); Write_String("\n");