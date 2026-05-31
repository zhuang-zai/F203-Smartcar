#ifndef __MPU_H_
#define __MPU_H_

#include "DEF.h"

typedef struct
{
    int16 acc_x;
    int16 acc_y;
    int16 acc_z;
    int16 gyro_x;
    int16 gyro_y;
    int16 gyro_z;
    int16 temp;
} IMU_Data_TypeDef;

IMU_Data_TypeDef* IMU_GetData(void);
int16 IMU_GetYawRate(void);

#endif