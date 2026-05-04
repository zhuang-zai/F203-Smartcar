#include "lcd.h"
#include "inductance.h"

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

    sprintf(txt, "L1:%03d ", inductance_values[0]);
    LCD_ShowStr_16(0, 2, txt, u16_RED_IPS, u16_BLACK);
    
    sprintf(txt, "L2:%03d ", inductance_values[1]);
    LCD_ShowStr_16(0, 3, txt, u16_RED_IPS, u16_BLACK);
    
    sprintf(txt, "M:%03d ", inductance_values[2]);
    LCD_ShowStr_16(0, 4, txt, u16_RED_IPS, u16_BLACK);
    
    sprintf(txt, "R2:%03d ", inductance_values[4]);
    LCD_ShowStr_16(0, 5, txt, u16_RED_IPS, u16_BLACK);
    
    sprintf(txt, "R1:%03d ", inductance_values[3]);
    LCD_ShowStr_16(0, 6, txt, u16_RED_IPS, u16_BLACK);
}

