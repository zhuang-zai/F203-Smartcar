#include "lcd.h"
#include "inductance.h"
#include "chassis.h"
#include "motor.h"
#include "IMU.h"
char txt[30];
int *inductance_values;

void Lcd_Display(void)
{
    // 使用 Motor_GetData() 和 Chassis_GetData() 获取只读数据指针，替代旧的多函数调用方式
    MotorData *mot = Motor_GetData();
    const Chassis_TypeDef *chs = Chassis_GetData();
    inductance_values = GetInductance();

    // 显示 5 路归一化电感值
    sprintf(txt, "L1:%03d L2:%03d", inductance_values[0],inductance_values[1]);
    LCD_ShowStr_16(0, 0, txt, u16_RED_IPS, u16_BLACK);
    
    sprintf(txt, "M:%03d ", inductance_values[2]);
    LCD_ShowStr_16(0, 1, txt, u16_RED_IPS, u16_BLACK);
    
    sprintf(txt, "R1:%03d R2:%03d", inductance_values[3],inductance_values[4]);
    LCD_ShowStr_16(0, 2, txt, u16_RED_IPS, u16_BLACK);
    
		
	// 使用 chs->current_deviation 直接读取已计算的偏差值，避免调用 Calculate_Deviation() 触发停车副作用
	sprintf(txt, "eleV:%04d  Dout:%04d", chs->current_deviation, chs->direction_output);
    LCD_ShowStr_16(0, 3, txt, u16_RED_IPS, u16_BLACK);

	// 使用 IMU_GetData() 获取陀螺仪只读指针
	sprintf(txt, "yawrate:%5d stop:%d", IMU_GetData()->gyro_z, mot->stop_flag);
	LCD_ShowStr_16(0, 4, txt, u16_RED_IPS, u16_BLACK);

	sprintf(txt,"LTS:%d  RTS:%d", chs->left_speed, chs->right_speed);
	LCD_ShowStr_16(0, 5, txt, u16_RED_IPS, u16_BLACK);

	sprintf(txt,"EL:%d  ER:%d", mot->encoder_L, mot->encoder_R);
	LCD_ShowStr_16(0, 6, txt, u16_RED_IPS, u16_BLACK);

	sprintf(txt,"LPWM:%04d RPWM:%04d", mot->left_pwm, mot->right_pwm);
	LCD_ShowStr_16(0, 7, txt, u16_RED_IPS, u16_BLACK);
}

