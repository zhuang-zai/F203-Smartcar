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

void main(void)
{
    System_Init();  /* 系统初始化 必须保留 */
    Global_IRQ_Enable(); // 使能全局中断
    GPIO_LED_Init();
    
    User_Init(); // 统一初始化所有外设和模块
		
    while (1)
    {
        LED_Ctrl(LED0, RVS);
		Lcd_Display();
		// 使用chassis控制: 外环差比和+方向PID，内环速度环PID
		Chassis_Control();
    }
}

