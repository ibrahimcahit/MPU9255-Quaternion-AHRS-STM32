/*
 * mpu9255.c
 *
 *  Created on: Dec 26, 2021
 *      Author: Ibrahim Ozdemir
 *      GitHub: ibrahimcahit   
 */

//Includes
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "tim.h"
#include "mpu9255.h"

const uint16_t i2c_timeout = 100;

// Set initial input parameters
enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

enum Mscale {
  MFS_14BITS = 0, // 0.6 mG per LSB
  MFS_16BITS      // 0.15 mG per LSB
};

//Specify sensor full scale
uint8_t Gscale = GFS_2000DPS;
uint8_t Ascale = AFS_16G;
uint8_t Mscale = MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution
uint8_t Mmode = 0x06;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read

float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors

int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
float magCalibration[3] = {0, 0, 0}; // Factory mag calibration and mag bias
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}, magBias[3] = {0, 0, 0}, magScale[3]  = {0, 0, 0};      // Bias corrections for gyro and accelerometer
float   SelfTest[6];    // holds results of gyro and accelerometer self test

float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)

//float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
//float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

float beta = 0.6045998;
float zeta = 0.0;

#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

float pitch, yaw, roll;
float deltat = 0.0f, sum = 0.0f;        // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;        // used to calculate integration interval

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

float lin_ax, lin_ay, lin_az;             // linear acceleration (acceleration with gravity component subtracted)
float a12, a22, a31, a32, a33;            // rotation matrix coefficients for Euler angles and gravity components

//===================================================================================================================
//====== MAIN INIT FUNCTION
//===================================================================================================================

uint8_t MPU9255_Init(I2C_HandleTypeDef *I2Cx){
	//pre-def. vars
	uint8_t readData;
	uint8_t writeData;
	printf("**************************** \r\n");
	printf("MPU9250 STM32 Implementation \r\n");
	printf("**************************** \r\n");

	//read MPU9255 WHOAMI
	HAL_I2C_Mem_Read(I2Cx, MPU9250_ADDRESS, WHO_AM_I_MPU9250, 1, &readData, 1, i2c_timeout);

	if (SerialDebugA) {printf("MPU -WHO AM I- is: (Must return 113) %d\r\n", readData);}

	if (readData == 113) {

		//Start by performing self test and reporting values
		MPU9250SelfTest(I2Cx, SelfTest);

		//Calibrate gyro and accelerometers, load biases in bias registers
		calibrateMPU9250(I2Cx, gyroBias, accelBias);
		HAL_Delay(1000);

		//init Gyro and Accelerometer
		initMPU9250(I2Cx);

		//enable Mag bypass
		writeData = 0x22;
		HAL_I2C_Mem_Write(I2Cx, MPU9250_ADDRESS, INT_PIN_CFG, 1, &writeData, 1, i2c_timeout);

		//Read the WHO_AM_I register of the magnetometer
		HAL_I2C_Mem_Read(I2Cx, AK8963_ADDRESS, AK8963_WHO_AM_I, 1, &readData, 1, i2c_timeout);// Read WHO_AM_I register for AK8963
		if (SerialDebugA) {printf("MAG -WHO AM I- is: (Must return 72) %d\r\n", readData);}
		HAL_Delay(1000);

		//Get magnetometer calibration from AK8963 ROM
		initAK8963(I2Cx, magCalibration);  // Initialize device for active mode read of magnetometer

		calibrateMag(I2Cx, magBias, magScale);

		HAL_Delay(1000);
		return 0;
	}
	return 1; // Loop forever if communication doesn't happen
}

void readAll(I2C_HandleTypeDef *I2Cx, MPU9255_t*DataStruct) {
	uint8_t Data;

	// If intPin goes high, all data registers have new data
	HAL_I2C_Mem_Read(I2Cx, MPU9250_ADDRESS, INT_STATUS, 1, &Data, 1, i2c_timeout);
	if (Data & 0x01) {  // On interrupt, check if data ready interrupt
		readAccelData(I2Cx, accelCount);  // Read the x/y/z adc values
		getAres();

		// Now we'll calculate the accleration value into actual g's
		ax = (float)accelCount[0]*aRes; // - accelBias[0];  // get actual g value, this depends on scale being set
		ay = (float)accelCount[1]*aRes; // - accelBias[1];
		az = (float)accelCount[2]*aRes; // - accelBias[2];

		DataStruct->AccelX = ax;
		DataStruct->AccelY = ay;
		DataStruct->AccelZ = az;

		readGyroData(I2Cx, gyroCount);  // Read the x/y/z adc values
		getGres();

		// Calculate the gyro value into actual degrees per second
		gx = (float)gyroCount[0]*gRes;  // get actual gyro value, this depends on scale being set
		gy = (float)gyroCount[1]*gRes;
		gz = (float)gyroCount[2]*gRes;

		DataStruct->GyroX = gx;
		DataStruct->GyroY = gy;
		DataStruct->GyroZ = gz;

		readMagData(I2Cx, magCount);  // Read the x/y/z adc values
		getMres();

		// Calculate the magnetometer values in milliGauss
		// Include factory calibration per data sheet and user environmental corrections
		mx = (float)magCount[0]*mRes*magCalibration[0] - magBias[0];  // get actual magnetometer value, this depends on scale being set
		my = (float)magCount[1]*mRes*magCalibration[1] - magBias[1];
		mz = (float)magCount[2]*mRes*magCalibration[2] - magBias[2];
		mx *= magScale[0];
		my *= magScale[1];
		mz *= magScale[2];

		DataStruct->MagX = mx;
		DataStruct->MagY = my;
		DataStruct->MagZ = mz;
	}

	//Now = __HAL_TIM_GET_COUNTER(&htim1);
	Now = HAL_GetTick();
	deltat = ((Now - lastUpdate)/1000.0f); // set integration time by time elapsed since last filter update
	lastUpdate = Now;
	sum += deltat; // sum for averaging filter update rate

	// Sensors x (y)-axis of the accelerometer/gyro is aligned with the y (x)-axis of the magnetometer;
	// the magnetometer z-axis (+ down) is misaligned with z-axis (+ up) of accelerometer and gyro.

	// Calculate quaternions based on Madgwick's filter
	//Since MPU9250's mag. and IMU modules are different and seperate (AK8963 and MPU6050), their...
	//...coordinate systems also different. So, to compensate this, order should be my - mx - mz
	//QuaternionUpdate(-ax, ay, az, gx*PI/180.0f, -gy*PI/180.0f, -gz*PI/180.0f,  my,  -mx, mz);
	QuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);

	// Convert quaternions to Euler angles
	a12 =   2.0f * (q[1] * q[2] + q[0] * q[3]);
	a22 =   q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
	a31 =   2.0f * (q[0] * q[1] + q[2] * q[3]);
	a32 =   2.0f * (q[1] * q[3] - q[0] * q[2]);
	a33 =   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];

	pitch = -asinf(a32);
	roll  = atan2f(a31, a33);
	yaw   = atan2f(a12, a22);
	pitch *= 180.0f / PI;
	yaw   *= 180.0f / PI;
	yaw   += 5.53f; // Declination

	if(yaw < 0) yaw   += 360.0f; // Ensure yaw stays between 0 and 360
	roll  *= 180.0f / PI;
	lin_ax = ax + a31;
	lin_ay = ay + a32;
	lin_az = az - a33;


	DataStruct->yaw = yaw;
	DataStruct->pitch = pitch;
	DataStruct->roll = roll;

	sum = 0;
	}

//==========================================================================================================
//====== FUNCTIONS TO READ AND WRITE DATA FROM REGISTERS AND ALSO INITS KALMAN AND QUATERNION FILTERS ======
//==========================================================================================================

void getMres() {
  switch (Mscale)
  {
  // Possible magnetometer scales (and their register bit settings) are:
  // 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
          mRes = 10.*4912./8190.; // Proper scale to return milliGauss
          break;
    case MFS_16BITS:
          mRes = 10.*4912./32760.0; // Proper scale to return milliGauss
          break;
  }
}

void getGres() {
  switch (Gscale)
  {
  // Possible gyro scales (and their register bit settings) are:
  // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
          gRes = 250.0/32768.0;
          break;
    case GFS_500DPS:
          gRes = 500.0/32768.0;
          break;
    case GFS_1000DPS:
          gRes = 1000.0/32768.0;
          break;
    case GFS_2000DPS:
          gRes = 2000.0/32768.0;
          break;
  }
}

void getAres() {
  switch (Ascale)
  {
  // Possible accelerometer scales (and their register bit settings) are:
  // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
          aRes = 2.0/32768.0;
          break;
    case AFS_4G:
          aRes = 4.0/32768.0;
          break;
    case AFS_8G:
          aRes = 8.0/32768.0;
          break;
    case AFS_16G:
          aRes = 16.0/32768.0;
          break;
  }
}

//read raw Accelerometer values from registers
void readAccelData(I2C_HandleTypeDef *I2Cx, int16_t * destination){
  uint8_t rawAccelData[6];  // x/y/z accel register data stored here
  HAL_I2C_Mem_Read(I2Cx, MPU9250_ADDRESS, ACCEL_XOUT_H, 1, &rawAccelData[0], 6, i2c_timeout); // Read the six raw data registers into data array
  destination[0] = ((int16_t)rawAccelData[0] << 8) | rawAccelData[1];  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawAccelData[2] << 8) | rawAccelData[3];
  destination[2] = ((int16_t)rawAccelData[4] << 8) | rawAccelData[5];

  if(SerialDebugB){
	printf("Acc X: %d\r\n", destination[0]);
	printf("Acc Y: %d\r\n", destination[1]);
	printf("Acc Z: %d\r\n", destination[2]);
	printf("-------------------------\r\n");
  }
}

//read raw Gyro values from registers
void readGyroData(I2C_HandleTypeDef *I2Cx, int16_t * destination){
  uint8_t rawGyroData[6];  // x/y/z gyro register data stored here
  HAL_I2C_Mem_Read(I2Cx, MPU9250_ADDRESS, GYRO_XOUT_H, 1, &rawGyroData[0], 6, i2c_timeout);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawGyroData[0] << 8) | rawGyroData[1];  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawGyroData[2] << 8) | rawGyroData[3];
  destination[2] = ((int16_t)rawGyroData[4] << 8) | rawGyroData[5];

  if(SerialDebugB){
	printf("Gyro X: %d\r\n", destination[0]);
	printf("Gyro Y: %d\r\n", destination[1]);
	printf("Gyro Z: %d\r\n", destination[2]);
	printf("---------------------------\r\n");
  }
}

void readMagData(I2C_HandleTypeDef *I2Cx, int16_t * destination){

	uint8_t readData;

	HAL_I2C_Mem_Read(I2Cx, AK8963_ADDRESS, AK8963_ST1, 1, &readData, 1, i2c_timeout);
	if( (readData & 0x01) == 0x01 ){
		uint8_t rawMagData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
		HAL_I2C_Mem_Read(I2Cx, AK8963_ADDRESS, AK8963_XOUT_L, 1, &rawMagData[0], 7, i2c_timeout);  // Read the six raw data and ST2 registers sequentially into data array
		uint8_t c = rawMagData[6];
		if(!(c & 0x08)) {
			destination[0] = ((int16_t)rawMagData[1] << 8) | rawMagData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
			destination[1] = ((int16_t)rawMagData[3] << 8) | rawMagData[2] ;  // Data stored as little Endian
			destination[2] = ((int16_t)rawMagData[5] << 8) | rawMagData[4] ;

			if(SerialDebugB){
				printf("Mag X: %d\r\n", destination[0]);
				printf("Mag Y: %d\r\n", destination[1]);
				printf("Mag Z: %d\r\n", destination[2]);
				printf("-------------------------\r\n");
			}
		}
	}
}

void initAK8963(I2C_HandleTypeDef *I2Cx, float * destination){
  //pre def. vars
  uint8_t writeData;

  //First extract the factory calibration for each magnetometer axis
  // x/y/z gyro calibration data stored here
  uint8_t rawMagCalData[3];

  //Power down magnetometer
  writeData = 0x00;
  HAL_I2C_Mem_Write(I2Cx, AK8963_ADDRESS, AK8963_CNTL, 1, &writeData, 1, i2c_timeout);
  HAL_Delay(100);

  writeData = 0x0F;
  HAL_I2C_Mem_Write(I2Cx, AK8963_ADDRESS, AK8963_CNTL, 1, &writeData, 1, i2c_timeout);// Enter Fuse ROM access mode
  HAL_Delay(100);


  HAL_I2C_Mem_Read(I2Cx, AK8963_ADDRESS, AK8963_ASAX, 1, &rawMagCalData[0], 3, i2c_timeout);// Read the x-, y-, and z-axis calibration values
  destination[0] =  (float)(rawMagCalData[0] - 128)/256. + 1.;   // Return x-axis sensitivity adjustment values, etc.
  destination[1] =  (float)(rawMagCalData[1] - 128)/256. + 1.;
  destination[2] =  (float)(rawMagCalData[2] - 128)/256. + 1.;

  if(SerialDebugA){
	printf("Mag cal off X: %f\r\n", destination[0]);
	printf("Mag cal off Y: %f\r\n", destination[1]);
	printf("Mag cal off Z: %f\r\n", destination[2]);
	printf("-------------------------\r\n");
  }

  writeData = 0x00;
  HAL_I2C_Mem_Write(I2Cx, AK8963_ADDRESS, AK8963_CNTL, 1, &writeData, 1, i2c_timeout);// Power down magnetometer
  HAL_Delay(100);

  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  writeData = Mscale << 4 | Mmode;
  HAL_I2C_Mem_Write(I2Cx, AK8963_ADDRESS, AK8963_CNTL, 1, &writeData, 1, i2c_timeout);// Set magnetometer data resolution and sample ODR

//writeData = 0x16;
//HAL_I2C_Mem_Write(I2Cx, AK8963_ADDRESS, AK8963_CNTL, 1, &writeData, 1, i2c_timeout);
  HAL_Delay(10);

  if(SerialDebugA){printf("MAG Init Succesful! \r\n");}
}

void calibrateMag(I2C_HandleTypeDef *I2Cx, float * dest1, float * dest2){

  uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
  int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};

  if (SerialDebugA){printf("Mag Calibration: Wave device in a figure eight until done!\r\n");}
  HAL_Delay(4000);

    // shoot for ~fifteen seconds of mag data
    if(Mmode == 0x02) sample_count = 128;  // at 8 Hz ODR, new mag data is available every 125 ms
    if(Mmode == 0x06) sample_count = 1500;  // at 100 Hz ODR, new mag data is available every 10 ms
   for(ii = 0; ii < sample_count; ii++) {
    readMagData(I2Cx, mag_temp);  // Read the mag data
    for (int jj = 0; jj < 3; jj++) {
      if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
    if(Mmode == 0x02) HAL_Delay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
    if(Mmode == 0x06) HAL_Delay(12);  // at 100 Hz ODR, new mag data is available every 10 ms
    }

    // Get hard iron correction
    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

    dest1[0] = (float) mag_bias[0]*mRes*magCalibration[0];  // save mag biases in G for main program
    dest1[1] = (float) mag_bias[1]*mRes*magCalibration[1];
    dest1[2] = (float) mag_bias[2]*mRes*magCalibration[2];

    // Get soft iron correction estimate
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0;

    dest2[0] = avg_rad/((float)mag_scale[0]);
    dest2[1] = avg_rad/((float)mag_scale[1]);
    dest2[2] = avg_rad/((float)mag_scale[2]);

    if (SerialDebugA){printf("Mag Calibration done!\r\n");}
}


void initMPU9250(I2C_HandleTypeDef *I2Cx){
	//pre def. vars
	uint8_t readData;
	uint8_t writeData;

	//Wake up device
	writeData = 0x00;
	HAL_I2C_Mem_Write(I2Cx, MPU9250_ADDRESS, PWR_MGMT_1, 1, &writeData, 1, i2c_timeout);
	HAL_Delay(100);

	writeData = 0x01;
	HAL_I2C_Mem_Write(I2Cx, MPU9250_ADDRESS, PWR_MGMT_1, 1, &writeData, 1, i2c_timeout);
	HAL_Delay(100);

	writeData = 0x03;
	HAL_I2C_Mem_Write(I2Cx, MPU9250_ADDRESS, CONFIG, 1, &writeData, 1, i2c_timeout);
	HAL_Delay(100);

	writeData = 0x04;
	HAL_I2C_Mem_Write(I2Cx, MPU9250_ADDRESS, SMPLRT_DIV, 1, &writeData, 1, i2c_timeout);
	HAL_Delay(100);

	HAL_I2C_Mem_Read(I2Cx, MPU9250_ADDRESS, GYRO_CONFIG, 1, &readData, 1, i2c_timeout);
	readData = readData & ~0x03; // Clear Fchoice bits [1:0]
	readData = readData & ~0x18; // Clear GFS bits [4:3]
	readData = readData | Gscale << 3; // Set full scale range for the gyro
	HAL_Delay(100);

	writeData = readData;
	HAL_I2C_Mem_Write(I2Cx, MPU9250_ADDRESS, GYRO_CONFIG, 1, &writeData, 1, i2c_timeout);
	HAL_Delay(100);

	HAL_I2C_Mem_Read(I2Cx, MPU9250_ADDRESS, ACCEL_CONFIG, 1, &readData, 1, i2c_timeout);
	readData = readData & ~0x18;  // Clear AFS bits [4:3]
	readData = readData | Ascale << 3; // Set full scale range for the accelerometer

	writeData = readData;
	HAL_I2C_Mem_Write(I2Cx, MPU9250_ADDRESS, ACCEL_CONFIG, 1, &writeData, 1, i2c_timeout);
	HAL_Delay(100);
	//**
	HAL_I2C_Mem_Read(I2Cx, MPU9250_ADDRESS, ACCEL_CONFIG2, 1, &readData, 1, i2c_timeout);
	readData = readData & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
	readData = readData | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz

	writeData = readData;
	HAL_I2C_Mem_Write(I2Cx, MPU9250_ADDRESS, ACCEL_CONFIG2, 1, &writeData, 1, i2c_timeout);
	HAL_Delay(100);

	if(SerialDebugA){printf("MPU Init Succesful! \r\n");}
}


// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void calibrateMPU9250(I2C_HandleTypeDef *I2Cx, float * dest1, float * dest2){
  //pre def. vars
  uint8_t writeData;

	uint8_t calibData[12]; // data array to hold accelerometer and gyro x, y, z, data
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

	// reset device
	writeData = 0x80;
	HAL_I2C_Mem_Write(I2Cx, MPU9250_ADDRESS, PWR_MGMT_1, 1, &writeData, 1, i2c_timeout);// Write a one to bit 7 reset bit; toggle reset device
	HAL_Delay(100);

	// get stable time source; Auto select clock source to be PLL gyroscope reference if ready
	// else use the internal oscillator, bits 2:0 = 001
	writeData = 0x01;
	HAL_I2C_Mem_Write(I2Cx, MPU9250_ADDRESS, PWR_MGMT_1, 1, &writeData, 1, i2c_timeout);
	writeData = 0x00;
	HAL_I2C_Mem_Write(I2Cx, MPU9250_ADDRESS, PWR_MGMT_2, 1, &writeData, 1, i2c_timeout);
	HAL_Delay(200);

	// Configure device for bias calculation
	writeData = 0x00;
	HAL_I2C_Mem_Write(I2Cx, MPU9250_ADDRESS, INT_ENABLE, 1, &writeData, 1, i2c_timeout);// Disable all interrupts
	writeData = 0x00;
	HAL_I2C_Mem_Write(I2Cx, MPU9250_ADDRESS, FIFO_EN, 1, &writeData, 1, i2c_timeout);// Disable FIFO
	writeData = 0x00;
	HAL_I2C_Mem_Write(I2Cx, MPU9250_ADDRESS, PWR_MGMT_1, 1, &writeData, 1, i2c_timeout);// Turn on internal clock source
	writeData = 0x00;
	HAL_I2C_Mem_Write(I2Cx, MPU9250_ADDRESS, I2C_MST_CTRL, 1, &writeData, 1, i2c_timeout);// Disable I2C master
	writeData = 0x00;
	HAL_I2C_Mem_Write(I2Cx, MPU9250_ADDRESS, USER_CTRL, 1, &writeData, 1, i2c_timeout);// Disable FIFO and I2C master modes
	writeData = 0x0C;
	HAL_I2C_Mem_Write(I2Cx, MPU9250_ADDRESS, USER_CTRL, 1, &writeData, 1, i2c_timeout);// Reset FIFO and DMP
	HAL_Delay(15);

	// Configure MPU6050 gyro and accelerometer for bias calculation
	writeData = 0x01;
	HAL_I2C_Mem_Write(I2Cx, MPU9250_ADDRESS, CONFIG, 1, &writeData, 1, i2c_timeout);// Set low-pass filter to 188 Hz
	writeData = 0x00;
	HAL_I2C_Mem_Write(I2Cx, MPU9250_ADDRESS, SMPLRT_DIV, 1, &writeData, 1, i2c_timeout);// Set sample rate to 1 kHz
	writeData = 0x00;
	HAL_I2C_Mem_Write(I2Cx, MPU9250_ADDRESS, GYRO_CONFIG, 1, &writeData, 1, i2c_timeout);// Set gyro full-scale to 250 degrees per second, maximum sensitivity
	writeData = 0x00;
	HAL_I2C_Mem_Write(I2Cx, MPU9250_ADDRESS, ACCEL_CONFIG, 1, &writeData, 1, i2c_timeout);// Set accelerometer full-scale to 2 g, maximum sensitivity

	uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
	uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	writeData = 0x40;
	HAL_I2C_Mem_Write(I2Cx, MPU9250_ADDRESS, USER_CTRL, 1, &writeData, 1, i2c_timeout);// Enable FIFO
	writeData = 0x78;
	HAL_I2C_Mem_Write(I2Cx, MPU9250_ADDRESS, FIFO_EN, 1, &writeData, 1, i2c_timeout);// Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
	HAL_Delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

	// At end of sample accumulation, turn off FIFO sensor read
	writeData = 0x00;
	HAL_I2C_Mem_Write(I2Cx, MPU9250_ADDRESS, FIFO_EN, 1, &writeData, 1, i2c_timeout);// Disable gyro and accelerometer sensors for FIFO
	HAL_I2C_Mem_Read(I2Cx, MPU9250_ADDRESS, FIFO_COUNTH, 1, &calibData[0], 2, i2c_timeout);// read FIFO sample count
	fifo_count = ((uint16_t)calibData[0] << 8) | calibData[1];
	packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

	for (ii = 0; ii < packet_count; ii++) {
		int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
		HAL_I2C_Mem_Read(I2Cx, MPU9250_ADDRESS, FIFO_R_W, 1, &calibData[0], 12, i2c_timeout);

		//Form signed 16-bit integer for each sample in FIFO
		accel_temp[0] = (int16_t) (((int16_t)calibData[0] << 8) | calibData[1]  ) ;
		accel_temp[1] = (int16_t) (((int16_t)calibData[2] << 8) | calibData[3]  ) ;
		accel_temp[2] = (int16_t) (((int16_t)calibData[4] << 8) | calibData[5]  ) ;
		gyro_temp[0]  = (int16_t) (((int16_t)calibData[6] << 8) | calibData[7]  ) ;
		gyro_temp[1]  = (int16_t) (((int16_t)calibData[8] << 8) | calibData[9]  ) ;
		gyro_temp[2]  = (int16_t) (((int16_t)calibData[10] << 8) | calibData[11]) ;

		//Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		accel_bias[0] += (int32_t) accel_temp[0];
		accel_bias[1] += (int32_t) accel_temp[1];
		accel_bias[2] += (int32_t) accel_temp[2];
		gyro_bias[0]  += (int32_t) gyro_temp[0];
		gyro_bias[1]  += (int32_t) gyro_temp[1];
		gyro_bias[2]  += (int32_t) gyro_temp[2];
	}

	//Normalize sums to get average count biases
	accel_bias[0] /= (int32_t) packet_count;
	accel_bias[1] /= (int32_t) packet_count;
	accel_bias[2] /= (int32_t) packet_count;
	gyro_bias[0]  /= (int32_t) packet_count;
	gyro_bias[1]  /= (int32_t) packet_count;
	gyro_bias[2]  /= (int32_t) packet_count;

	//Remove gravity from the z-axis accelerometer bias calculation
	if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}
	else {accel_bias[2] += (int32_t) accelsensitivity;}

	//Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	calibData[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	calibData[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
	calibData[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
	calibData[3] = (-gyro_bias[1]/4)       & 0xFF;
	calibData[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
	calibData[5] = (-gyro_bias[2]/4)       & 0xFF;

	//Push gyro biases to hardware registers
	writeData = calibData[0];
	HAL_I2C_Mem_Write(I2Cx, MPU9250_ADDRESS, XG_OFFSET_H, 1, &writeData, 1, i2c_timeout);
	writeData = calibData[1];
	HAL_I2C_Mem_Write(I2Cx, MPU9250_ADDRESS, XG_OFFSET_L, 1, &writeData, 1, i2c_timeout);
	writeData = calibData[2];
	HAL_I2C_Mem_Write(I2Cx, MPU9250_ADDRESS, YG_OFFSET_H, 1, &writeData, 1, i2c_timeout);
	writeData = calibData[3];
	HAL_I2C_Mem_Write(I2Cx, MPU9250_ADDRESS, YG_OFFSET_L, 1, &writeData, 1, i2c_timeout);
	writeData = calibData[4];
	HAL_I2C_Mem_Write(I2Cx, MPU9250_ADDRESS, ZG_OFFSET_H, 1, &writeData, 1, i2c_timeout);
	writeData = calibData[5];
	HAL_I2C_Mem_Write(I2Cx, MPU9250_ADDRESS, ZG_OFFSET_L, 1, &writeData, 1, i2c_timeout);

	//Output scaled gyro biases for display in the main program
	dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity;
	dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
	dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

	if(SerialDebugA){
		float gyroBiasX = (float) gyro_bias[0]/(float) gyrosensitivity;
		float gyroBiasY = (float) gyro_bias[1]/(float) gyrosensitivity;
		float gyroBiasZ = (float) gyro_bias[2]/(float) gyrosensitivity;

		printf("Gyro bias X: %f\r\n", gyroBiasX);
		printf("Gyro bias Y: %f\r\n", gyroBiasY);
		printf("Gyro bias Z: %f\r\n", gyroBiasZ);

		printf("-------------------------\r\n");
	}

	//Construct the accelerometer biases for push to the hardware accelerometer bias registers.
	int32_t accel_bias_reg[3] = {0, 0, 0}; //A place to hold the factory accelerometer trim biases
	HAL_I2C_Mem_Read(I2Cx, MPU9250_ADDRESS, XA_OFFSET_H, 1, &calibData[0], 2, i2c_timeout); //Read factory accelerometer trim values
	accel_bias_reg[0] = (int32_t) (((int16_t)calibData[0] << 8) | calibData[1]);
	HAL_I2C_Mem_Read(I2Cx, MPU9250_ADDRESS, YA_OFFSET_H, 1, &calibData[0], 2, i2c_timeout);
	accel_bias_reg[1] = (int32_t) (((int16_t)calibData[0] << 8) | calibData[1]);
	HAL_I2C_Mem_Read(I2Cx, MPU9250_ADDRESS, ZA_OFFSET_H, 1, &calibData[0], 2, i2c_timeout);
	accel_bias_reg[2] = (int32_t) (((int16_t)calibData[0] << 8) | calibData[1]);

	//Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	uint32_t mask = 1uL;
	//Define array to hold mask bit for each accelerometer bias axis
	uint8_t mask_bit[3] = {0, 0, 0};

	for(ii = 0; ii < 3; ii++) {
		//If temperature compensation bit is set, record that fact in mask_bit
		if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01;
	}

	//Construct total accelerometer bias, including calculated average accelerometer bias from above
	accel_bias_reg[0] -= (accel_bias[0]/8); //Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
	accel_bias_reg[1] -= (accel_bias[1]/8);
	accel_bias_reg[2] -= (accel_bias[2]/8);

	calibData[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	calibData[1] = (accel_bias_reg[0])      & 0xFF;
	calibData[1] = calibData[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	calibData[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	calibData[3] = (accel_bias_reg[1])      & 0xFF;
	calibData[3] = calibData[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	calibData[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	calibData[5] = (accel_bias_reg[2])      & 0xFF;
	calibData[5] = calibData[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

	//Push accelerometer biases to hardware registers
	writeData = calibData[0];
	HAL_I2C_Mem_Write(I2Cx, MPU9250_ADDRESS, XA_OFFSET_H, 1, &writeData, 1, i2c_timeout);
	writeData = calibData[1];
	HAL_I2C_Mem_Write(I2Cx, MPU9250_ADDRESS, XA_OFFSET_L, 1, &writeData, 1, i2c_timeout);
	writeData = calibData[2];
	HAL_I2C_Mem_Write(I2Cx, MPU9250_ADDRESS, YA_OFFSET_H, 1, &writeData, 1, i2c_timeout);
	writeData = calibData[3];
	HAL_I2C_Mem_Write(I2Cx, MPU9250_ADDRESS, YA_OFFSET_L, 1, &writeData, 1, i2c_timeout);
	writeData = calibData[4];
	HAL_I2C_Mem_Write(I2Cx, MPU9250_ADDRESS, ZA_OFFSET_H, 1, &writeData, 1, i2c_timeout);
	writeData = calibData[5];
	HAL_I2C_Mem_Write(I2Cx, MPU9250_ADDRESS, ZA_OFFSET_L, 1, &writeData, 1, i2c_timeout);

	//Output scaled gyro biases for display in the main program
	dest2[0] = (float) accel_bias[0]/(float) accelsensitivity;
	dest2[1] = (float) accel_bias[1]/(float) accelsensitivity;
	dest2[2] = (float) accel_bias[2]/(float) accelsensitivity;

	if(SerialDebugA){
		float accelBiasX = (float) accel_bias[0]/(float) accelsensitivity;
		float accelBiasY = (float) accel_bias[1]/(float) accelsensitivity;
		float accelBiasZ = (float) accel_bias[2]/(float) accelsensitivity;

		printf("Accel bias X: %f\r\n", accelBiasX);
		printf("Accel bias Y: %f\r\n", accelBiasY);
		printf("Accel bias Z: %f\r\n", accelBiasZ);

		printf("-------------------------\r\n");
	}
}


// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9250SelfTest(I2C_HandleTypeDef *I2Cx, float * destination) {
	uint8_t writeData;

	uint8_t rawTestData[6] = {0, 0, 0, 0, 0, 0};
	uint8_t selfTest[6];
	int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
	float factoryTrim[6];
	uint8_t FS = 0;

	writeData = 0x00;
	HAL_I2C_Mem_Write(I2Cx, MPU9250_ADDRESS, SMPLRT_DIV, 1, &writeData, 1, i2c_timeout);// Set gyro sample rate to 1 kHz
	writeData = 0x02;
	HAL_I2C_Mem_Write(I2Cx, MPU9250_ADDRESS, CONFIG, 1, &writeData, 1, i2c_timeout);// Set gyro sample rate to 1 kHz and DLPF to 92 Hz
	writeData = FS<<3;
	HAL_I2C_Mem_Write(I2Cx, MPU9250_ADDRESS, GYRO_CONFIG, 1, &writeData, 1, i2c_timeout);// Set full scale range for the gyro to 250 dps
	writeData = 0x02;
	HAL_I2C_Mem_Write(I2Cx, MPU9250_ADDRESS, ACCEL_CONFIG2, 1, &writeData, 1, i2c_timeout);// Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
	writeData = FS<<3;
	HAL_I2C_Mem_Write(I2Cx, MPU9250_ADDRESS, ACCEL_CONFIG, 1, &writeData, 1, i2c_timeout);// Set full scale range for the accelerometer to 2 g

	//get average current values of gyro and acclerometer
	for( int ii = 0; ii < 200; ii++) {

		HAL_I2C_Mem_Read(I2Cx, MPU9250_ADDRESS, ACCEL_XOUT_H, 1, &rawTestData[0], 6, i2c_timeout);// Read the six raw data registers into data array
		aAvg[0] += (int16_t)(((int16_t)rawTestData[0] << 8) | rawTestData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		aAvg[1] += (int16_t)(((int16_t)rawTestData[2] << 8) | rawTestData[3]) ;
		aAvg[2] += (int16_t)(((int16_t)rawTestData[4] << 8) | rawTestData[5]) ;

		HAL_I2C_Mem_Read(I2Cx, MPU9250_ADDRESS, GYRO_XOUT_H, 1, &rawTestData[0], 6, i2c_timeout);// Read the six raw data registers sequentially into data array
		gAvg[0] += (int16_t)(((int16_t)rawTestData[0] << 8) | rawTestData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		gAvg[1] += (int16_t)(((int16_t)rawTestData[2] << 8) | rawTestData[3]) ;
		gAvg[2] += (int16_t)(((int16_t)rawTestData[4] << 8) | rawTestData[5]) ;
	}

	//Get average of 200 values and store as average current readings
	for (int ii =0; ii < 3; ii++) {
		aAvg[ii] /= 200;
		gAvg[ii] /= 200;
	}

	//Configure the accelerometer for self-test
	writeData = 0xE0;
	HAL_I2C_Mem_Write(I2Cx, MPU9250_ADDRESS, ACCEL_CONFIG, 1, &writeData, 1, i2c_timeout);// Enable self test on all three axes and set accelerometer range to +/- 2 g
	writeData = 0xE0;
	HAL_I2C_Mem_Write(I2Cx, MPU9250_ADDRESS, GYRO_CONFIG, 1, &writeData, 1, i2c_timeout);// Enable self test on all three axes and set gyro range to +/- 250 degrees/s
	HAL_Delay(25);  // Delay a while to let the device stabilize

	//get average self-test values of gyro and acclerometer
	for( int ii = 0; ii < 200; ii++) {

		HAL_I2C_Mem_Read(I2Cx, MPU9250_ADDRESS, ACCEL_XOUT_H, 1, &rawTestData[0], 6, i2c_timeout);// Read the six raw data registers into data array
		aSTAvg[0] += (int16_t)(((int16_t)rawTestData[0] << 8) | rawTestData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		aSTAvg[1] += (int16_t)(((int16_t)rawTestData[2] << 8) | rawTestData[3]) ;
		aSTAvg[2] += (int16_t)(((int16_t)rawTestData[4] << 8) | rawTestData[5]) ;

		HAL_I2C_Mem_Read(I2Cx, MPU9250_ADDRESS, GYRO_XOUT_H, 1, &rawTestData[0], 6, i2c_timeout);// Read the six raw data registers sequentially into data array
		gSTAvg[0] += (int16_t)(((int16_t)rawTestData[0] << 8) | rawTestData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		gSTAvg[1] += (int16_t)(((int16_t)rawTestData[2] << 8) | rawTestData[3]) ;
		gSTAvg[2] += (int16_t)(((int16_t)rawTestData[4] << 8) | rawTestData[5]) ;
	}

	//Get average of 200 values and store as average self-test readings
	for (int ii =0; ii < 3; ii++) {
		aSTAvg[ii] /= 200;
		gSTAvg[ii] /= 200;
	}

	//Configure the gyro and accelerometer for normal operation
	writeData = 0x00;
	HAL_I2C_Mem_Write(I2Cx, MPU9250_ADDRESS, ACCEL_CONFIG, 1, &writeData, 1, i2c_timeout);
	writeData = 0x00;
	HAL_I2C_Mem_Write(I2Cx, MPU9250_ADDRESS, GYRO_CONFIG, 1, &writeData, 1, i2c_timeout);
	HAL_Delay(25);  // Delay a while to let the device stabilize

	//Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
	HAL_I2C_Mem_Read(I2Cx, MPU9250_ADDRESS, SELF_TEST_X_ACCEL, 1, &selfTest[0], 1, i2c_timeout);// X-axis accel self-test results
	HAL_I2C_Mem_Read(I2Cx, MPU9250_ADDRESS, SELF_TEST_Y_ACCEL, 1, &selfTest[1], 1, i2c_timeout);// Y-axis accel self-test results
	HAL_I2C_Mem_Read(I2Cx, MPU9250_ADDRESS, SELF_TEST_Z_ACCEL, 1, &selfTest[2], 1, i2c_timeout);// Z-axis accel self-test results
	HAL_I2C_Mem_Read(I2Cx, MPU9250_ADDRESS, SELF_TEST_X_GYRO, 1, &selfTest[3], 1, i2c_timeout);// X-axis gyro self-test results
	HAL_I2C_Mem_Read(I2Cx, MPU9250_ADDRESS, SELF_TEST_Y_GYRO, 1, &selfTest[4], 1, i2c_timeout);// Y-axis gyro self-test results
	HAL_I2C_Mem_Read(I2Cx, MPU9250_ADDRESS, SELF_TEST_Z_GYRO, 1, &selfTest[5], 1, i2c_timeout);// Z-axis gyro self-test results

	//Retrieve factory self-test value from self-test code reads
	factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
	factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
	factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
	factoryTrim[3] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
	factoryTrim[4] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
	factoryTrim[5] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation

	uint32_t testResults[6];

	//Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
	//To get percent, must multiply by 100
	for (int i = 0; i < 3; i++) {
		testResults[i]   = 100.0*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i] - 100.;   // Report percent differences
		testResults[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3] - 100.; // Report percent differences
	}

	if(SerialDebugA){
		float testResultAccelX = testResults[0];
		float testResultAccelY = testResults[1];
		float testResultAccelZ = testResults[2];
		float testResultGyroX = testResults[3];
		float testResultGyroY = testResults[4];
		float testResultGyroZ = testResults[5];

		printf("Accel Test X: %f\r\n", testResultAccelX);
		printf("Accel Test Y: %f\r\n", testResultAccelY);
		printf("Accel Test Z: %f\r\n", testResultAccelZ);
		printf("Gyro Test X: %f\r\n", testResultGyroX);
		printf("Gyro Test Y: %f\r\n", testResultGyroY);
		printf("Gyro Test Z: %f\r\n", testResultGyroZ);
		printf("-------------------------\r\n");
	}

   for (int i = 0; i < 3; i++) {
     destination[i]   = 100.0*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i] - 100.;   // Report percent differences
     destination[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3] - 100.; // Report percent differences
   }
}

void QuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz){
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
    float norm;
    float hx, hy, _2bx, _2bz;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4;

    // Auxiliary variables to avoid repeated arithmetic
    float _2q1mx;
    float _2q1my;
    float _2q1mz;
    float _2q2mx;
    float _4bx;
    float _4bz;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    // Normalise accelerometer measurement
    norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = sqrtf(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    _2q1mx = 2.0f * q1 * mx;
    _2q1my = 2.0f * q1 * my;
    _2q1mz = 2.0f * q1 * mz;
    _2q2mx = 2.0f * q2 * mx;
    hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
    hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
    norm = 1.0f/norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    // Compute rate of change of quaternion
    qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
    qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
    qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
    qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

    // Integrate to yield quaternion
    q1 += qDot1 * deltat;
    q2 += qDot2 * deltat;
    q3 += qDot3 * deltat;
    q4 += qDot4 * deltat;
    norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f/norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;

}
