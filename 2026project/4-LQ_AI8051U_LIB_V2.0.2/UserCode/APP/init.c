#include "init.h"
#include "lcd.h"
#include "motor.h"
#include "chassis.h"
#include "LQ_MotorServo.h"
#include "LQ_IPSLCD.h"
#include "LQ_TIMER.h"
#include "LQ_HI2C_Gyro.h"
#include "pid.h"
#include "inductance.h"

/**
 * @brief 统一初始化所有外设和模块
 * 包含: ADC, LCD, Timer, Motor, PID, Chassis等
 */
void User_Init(void)
{
    //LQ写的
    // 1. ADC初始化
    ADC_Init();
    // 2. LCD初始化
    IPS_LCD_Init();
    LCD_CLS(u16_BLACK);
    // 3. 定时器初始化 - 2ms周期
    TIM_Init_ms(Timer0, 2);
    TIM_Init_ms(Timer1, 2);
    // 4. 电机初始化
    Motor_Init(Motor_FREQ);
    // 5. 陀螺仪初始化（可选）
    // lsm6dsr_init();  //软件陀螺仪初始化
    LQ_HARD_SPI_LSM60DSR_Init(); // 硬件陀螺仪初始化

    //下面是我自己写的
    //PID参数初始化（速度环+方向环）
    PID_AllInit();
    //底盘初始化
    Chassis_Init();
    delay_ms(100);
}