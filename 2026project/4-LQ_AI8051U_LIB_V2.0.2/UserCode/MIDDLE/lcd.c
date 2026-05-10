#include "lcd.h"
#include "inductance.h"
#include "chassis.h"

char txt[30];
int *inductance_values; // 用于存储电感值的全局变量

/**
 * @brief 显示电磁传感器数据
 * 显示归一化后的五个电感值
 * @param values 电感值数组 [L1, L2, M, R1, R2]
 */
void Lcd_Display(void)
{
    inductance_values = GetInductance(); // 将只读电感值赋值给局部变量

    sprintf(txt, "L1:%03d L2:%03d", inductance_values[0],inductance_values[1]);
    LCD_ShowStr_16(0, 1, txt, u16_RED_IPS, u16_BLACK);
    
    sprintf(txt, "M:%03d ", inductance_values[2]);
    LCD_ShowStr_16(0, 2, txt, u16_RED_IPS, u16_BLACK);
    
    sprintf(txt, "R1:%03d R2:%03d", inductance_values[3],inductance_values[4]);
    LCD_ShowStr_16(0, 3, txt, u16_RED_IPS, u16_BLACK);
    
		
		/*偏差*/
		sprintf(txt, "eleV:%04d  Dout:%04d", Calculate_Deviation(),direction_output);
    LCD_ShowStr_16(0, 4, txt, u16_RED_IPS, u16_BLACK);
		/*偏航角*/
		sprintf(txt, "yawrate:%04d",actual_yaw_rate);
		LCD_ShowStr_16(0, 5, txt, u16_RED_IPS, u16_BLACK);
		/*左右目标速度*/
		sprintf(txt,"LTS:%d  RTS:%d",chassis.left_speed,chassis.right_speed);
		LCD_ShowStr_16(0, 6, txt, u16_RED_IPS, u16_BLACK);
		
}

