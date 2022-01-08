/*
 * mpu9255.h
 *
 *  Created on: Dec 26, 2021
 *      Author: Ibrahim Ozdemir
 *      GitHub: ibrahimcahit
 */

#ifndef INC_9255_H_
#define INC_9255_H_

#endif /* INC_9255_H_ */

#include <stdint.h>
#include "i2c.h"
#include "mpu9255_defs.h"

typedef struct
{
    float AccelX;
    float AccelY;
    float AccelZ;

    float GyroX;
    float GyroY;
    float GyroZ;

    float MagX;
    float MagY;
    float MagZ;

    float pitch;
    float roll;
    float yaw;

} MPU9255_t;

uint8_t MPU9255_Init(I2C_HandleTypeDef *I2Cx);

void readAll(I2C_HandleTypeDef *I2Cx, MPU9255_t*DataStruct);

void getMres();
void getGres();
void getAres();

void readAccelData(I2C_HandleTypeDef *I2Cx, int16_t * destination);
void readGyroData(I2C_HandleTypeDef *I2Cx, int16_t * destination);
void readMagData(I2C_HandleTypeDef *I2Cx, int16_t * destination);

void initAK8963(I2C_HandleTypeDef *I2Cx, float * destination);
void initMPU9250(I2C_HandleTypeDef *I2Cx);

void calibrateMPU9250(I2C_HandleTypeDef *I2Cx, float * dest1, float * dest2);
void calibrateMag(I2C_HandleTypeDef *I2Cx, float * dest1, float * dest2);

void MPU9250SelfTest(I2C_HandleTypeDef *I2Cx, float * destination);

void QuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
