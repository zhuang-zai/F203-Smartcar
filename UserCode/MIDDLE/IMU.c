#include "IMU.h"
#include "LQ_LSM6DSR_Hard.h"

static IMU_Data_TypeDef imu_data;

// 将陀螺仪原始数据变量定义在文件作用域，IMU_GetYawRate() 直接读取缓存值，避免重复调用硬件通信
static int16 ax, ay, az, gx, gy, gz;

IMU_Data_TypeDef* IMU_GetData(void)
{
#ifdef HARDWARE_SPI
    LSM6DSR_Read_Data(&ax, &ay, &az, &gx, &gy, &gz);
#elif defined HARDWARE_IIC
    LQ_HARD_IIC_LSM60DSR_Read(&ax, &ay, &az, &gx, &gy, &gz);
#else
    ax = ay = az = gx = gy = gz = 0;
#endif

    // 将文件作用域变量拷贝到返回结构体
    imu_data.acc_x = ax;
    imu_data.acc_y = ay;
    imu_data.acc_z = az;
    imu_data.gyro_x = gx;
    imu_data.gyro_y = gy;
    imu_data.gyro_z = gz;
    
    return &imu_data;
}

// 直接读取文件作用域的 gz 缓存值，避免重复调用硬件 SPI/I2C 通信
int16 IMU_GetYawRate(void)
{
    return gz;
}
