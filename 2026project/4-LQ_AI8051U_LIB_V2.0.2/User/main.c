/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 【平    台】北京龙邱智能科技STC32位核心板
 【编    写】龙邱科技
 【E-mail  】chiusir@163.com
 【软件版本】V1.1 版权所有，单位使用请先联系授权
 【相关信息参考下列地址】
 【网    站】http://www.lqist.cn
 【淘宝店铺】http://longqiu.taobao.com
 --------------------------------------------------------------------------------
 【  IDE  】 keil C251 V5.60
 【Target 】 STC32G/STC8051U/AI8051U 32位模式
 【SYS CLK】 40 MHz使用内部晶振
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

// 下载时, 选择时钟频率 与"config.h"中宏定义保持一致 默认使用40M主频


#include "include.h"
#include "init.h"
#include "motor.h"

volatile bit flag_200ms_lcd = 0;

void main(void)
{
//	int vofa_timer = 0;
    System_Init();  /* 系统初始化 必须保留 */
    Global_IRQ_Enable(); // 使能全局中断
    GPIO_LED_Init();
    
    User_Init(); // 统一初始化所有外设和模块
    while (1)
    {
			//BLmotor_Ctrl_w1(1200)
			Lcd_Display();
			/*vofa 用于速度环PID调参*/
//			vofa_timer++;
//			if(vofa_timer >= 2)
//			{
//				vofa_timer = 0;
//				printf("%d,%d,%d\n", vofa_target_speed, vofa_current_speed, vofa_out_pwm);
//			}
			LED_Ctrl(LED0, RVS);
    }
}

