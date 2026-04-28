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

void main(void)
{
    System_Init();  /* 系统初始化 必须保留 */
    Global_IRQ_Enable(); // 使能全局中断
    GPIO_LED_Init();
		/*以下为手动添加*/
		ADC_Init();  //电感采集初始化
		IPS_LCD_Init();  //LCD初始化
		LCD_CLS(u16_BLACK);
		TIM_Init_ms(Timer1, 2);  // 定时器1中断初始化
		TIM_Init_ms(Timer0, 3);  // 定时器0中断初始化
		//Motor_Init(Motor_FREQ);  // 电机初始化
		//Timer_EncInit(Timer3|Timer4);  // 编码器初始化
		//lsm6dsr_init();  //软件陀螺仪初始化
		LQ_HARD_SPI_LSM60DSR_Init(); // 硬件陀螺仪初始化
		/*以上为手动添加*/
    printf("USER Init OK  \n"); // printf选择 UART1~UART4（在STC32G_UART.h 中宏定义）
		
    while (1)
    {
        LED_Ctrl(LED0, RVS);
        //delay_ms(100); // 100ms
    }
}

