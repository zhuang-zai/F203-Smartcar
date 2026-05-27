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
		//编码器初始化 将 Timer3 和 Timer4 配置为外部脉冲计数模式
    Timer_EncInit(Timer3 | Timer4);
    // 3. 定时器初始化 
    TIM_Init_ms(Timer0, 1);  //电感采集
    TIM_Init_ms(Timer1, 2);  //控制算法
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
		left_pwm = 0;
    right_pwm = 0;
}

/**
 * @brief 按键发车等待函数 (V2.0 安全倒计时版)
 * @note 必须按下并松开 GO 按键，随后延时 1 秒才会发车
 */
void Wait_For_Start(void)
{
    // 初始化 P2_0 为上拉输入模式
    gpio_init_pin(P2_0, GPIO_Mode_IPU);
    
    // 等待按键按下
    while (gpio_read_pin(P2_0) == 1);
    
    // 按下消抖
    delay_ms(20); 
    
    // 确认按下
    if (gpio_read_pin(P2_0) == 0)
    {
        // 等待按键松开（核心防刮手逻辑：你不松手，它绝对不走）
        while (gpio_read_pin(P2_0) == 0); 
        
        // 松手消抖
        delay_ms(20);
        
        LED_Ctrl(LED0, ON); 
        delay_ms(500);
        
        LED_Ctrl(LED0, OFF);
        delay_ms(500);
        
        stop_flag = 0;
    }
}