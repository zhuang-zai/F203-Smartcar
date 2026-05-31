#include "init.h"
#include "lcd.h"
#include "motor.h"
#include "chassis.h"
#include "negative_pressure_fan.h"
#include "LQ_MotorServo.h"
#include "LQ_IPSLCD.h"
#include "LQ_TIMER.h"
#include "LQ_HI2C_Gyro.h"
#include "pid.h"
#include "inductance.h"
// 新增：包含调试模块头文件，用于串口调参和 VOFA+ 调试
#include "debug.h"

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
		//编码器初始化 将 Timer3 和 Timer4 配置为外部脉冲计数模式
    Timer_EncInit(Timer3 | Timer4);
    // 3. 定时器初始化
    TIM_Init_ms(Timer0, 1);  //电感采集
    TIM_Init_ms(Timer1, 2);  //控制算法
    TIM_Init_ms(Timer2, 20); //调试数据发送(50Hz)
		//TIM_Init_ms(Timer11,200);  //显示屏
    // 4. 电机初始化
    Motor_Init(Motor_FREQ);
    // 5. 陀螺仪初始化（可选）
    // lsm6dsr_init();  //软件陀螺仪初始化
    LQ_HARD_SPI_LSM60DSR_Init(); // 硬件陀螺仪初始化
		//负压风扇初始化
		BLmotor_Init_1(Servo_FREQ); // 频率为50Hz，此处频率需要与舵机频率保持一致,满占空比20000
		BLmotor_Ctrl_w1(900);
    //下面是我自己写的
    //PID参数初始化（速度环+方向环）
    PID_AllInit();
    //底盘初始化
    Chassis_Init();
    //调试模块初始化
    Debug_Init();
    //负压风扇
    Fan_Init(Servo_FREQ);
    Fan_SetSpeed(FAN_MODE_1);
    delay_ms(100);
}