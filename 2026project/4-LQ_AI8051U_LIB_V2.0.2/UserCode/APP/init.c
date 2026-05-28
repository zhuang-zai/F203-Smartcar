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
		
		ADC_Key_Init();
		
    // 2. LCD初始化
    IPS_LCD_Init();
    LCD_CLS(u16_BLACK);
		//编码器初始化 将 Timer3 和 Timer4 配置为外部脉冲计数模式
    Timer_EncInit(Timer3 | Timer4);
    // 3. 定时器初始化 
    TIM_Init_ms(Timer0, 1);  //电感采集
    TIM_Init_ms(Timer1, 2);  //控制算法
		TIM_Init_ms(Timer11,100);  //显示屏
    // 4. 电机初始化
    Motor_Init(Motor_FREQ);
    // 5. 陀螺仪初始化（可选）
    // lsm6dsr_init();  //软件陀螺仪初始化
    LQ_HARD_SPI_LSM60DSR_Init(); // 硬件陀螺仪初始化
	
		//负压风扇初始化
		BLmotor_Init_1(Servo_FREQ); // 频率为50Hz，此处频率需要与舵机频率保持一致,满占空比20000
    target_fan_pwm = 1200;
    current_fan_pwm = 900;
    BLmotor_Ctrl_w1(current_fan_pwm); // 以最低待机转速上电
  
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
    while (gpio_read_pin(P2_0) == 1)
		{
			if (flag_200ms_lcd == 1)
        {
            flag_200ms_lcd = 0;
            Lcd_Display();
            Key_Tuning_Task(); // 顺便支持发车前调参
        }
		}
    
    // 按下消抖
    delay_ms(20); 
    
    // 确认按下
    if (gpio_read_pin(P2_0) == 0)
    {
        // 等待按键松开（核心防刮手逻辑：你不松手，它绝对不走）
        while (gpio_read_pin(P2_0) == 0); 
        
        // 松手消抖
        delay_ms(20);
        
				// =====================================
        // 🌪️ 起飞预热：改变期望值，让现值平滑追踪
        // =====================================
        
        // 阻塞式追踪：只要现值还没达到期望值，就继续加
        while(current_fan_pwm < target_fan_pwm) 
        {
            current_fan_pwm += 20; 
            BLmotor_Ctrl_w1(current_fan_pwm);
            delay_ms(10); 
        }
			
        LED_Ctrl(LED0, ON); 
        delay_ms(500);
        LED_Ctrl(LED0, OFF);
        
        
        stop_flag = 0;
    }
}

void Emergency_Stop_Task(void)
{
    static uint8 key_press_cnt = 0; // 连续按下计数器

    // 如果系统已经是停车状态，就没必要检测急停了
    if (stop_flag == 1) 
    {
        key_press_cnt = 0;
        return;
    }

    // 检测 P2_0 是否被按下 (0为按下)
    if (gpio_read_pin(P2_0) == 0)
    {
//        key_press_cnt++;
//        
//        // 连续检测到按下大约 10 次 (相当于滤除了极短的震动或静电干扰)
//        if (key_press_cnt >= 100) 
//        {
            stop_flag = 1;     // 🚀 触发全局急停，底层电机和风扇会立刻被锁死！
//            key_press_cnt = 0; // 清零计数器
//        }
    }
//    else
//    {
//        // 只要期间有哪怕一次松开，或者只是电磁毛刺干扰，就立刻清零重新计数
//        key_press_cnt = 0; 
//    }
}

/**
 * @brief 按键调参后台任务 (非阻塞、防连加)
 * @note 放在 while(1) 中轮询
 */
/**
 * @brief 按键调参后台任务 (包含 P, D, 目标角速度 的动态调节)
 */
void Key_Tuning_Task(void)
{
    u8 rent_key = ADKey_Scan(); 
    
    if (rent_key != KEY_NONE)   
    {
        const PID_TypeDef* yaw_pid = PID_GetController(3);
        float temp_kp = yaw_pid->Kp;
        float temp_kd = yaw_pid->Kd;
        int16 temp_target = target_yaw_rate; // 拿到当前的目标角速度

        // 🌟 智能步进换算：如果是精度微调(0.01)，角速度每次加减 10；如果是粗调(0.02)，角速度加减 50
        int16 target_step = (tune_step < 0.015f) ? 10 : 50; 

        switch (rent_key)
        {
            case KEY_PRESS:   // 1. 中间按压键：三档切换 (0:Kp -> 1:Target -> 2:Kd -> 回到0)
                if (selected_param == 0) selected_param = 1;
                else if (selected_param == 1) selected_param = 2;
                else selected_param = 0;
                break;

            case KEY_UP:      // 2. 向上键：参数增加
                if (selected_param == 0)      temp_kp += tune_step;
                else if (selected_param == 1) temp_target += target_step; // 角速度增加
                else                          temp_kd += tune_step;
                break;

            case KEY_DOWN:    // 3. 向下键：参数减少
                if (selected_param == 0)      temp_kp -= tune_step;
                else if (selected_param == 1) temp_target -= target_step; // 角速度减小 (允许为负数，代表反转)
                else                          temp_kd -= tune_step;
                
                // P和D绝对不能为负数，但 target_yaw_rate 可以为负，所以不限制 temp_target
                if (temp_kp < 0.0f) temp_kp = 0.0f;
                if (temp_kd < 0.0f) temp_kd = 0.0f;
                break;

            case KEY_LEFT:    // 4. 向左键：高精度微调
                tune_step = 0.01f;
                break;

            case KEY_RIGHT:   // 5. 向右键：大幅度粗调
                tune_step = 0.02f;
                break;
        }

        // 重新刷进 RAM 和全局变量中
        PID_SetKp(3, temp_kp);
        PID_SetKd(3, temp_kd);
        target_yaw_rate = temp_target; 
    }
}