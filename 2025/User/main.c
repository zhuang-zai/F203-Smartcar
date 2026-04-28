/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 【平    台】北京龙邱智能科技STC32位核心板
 【编    写】龙邱科技
 【E-mail  】chiusir@163.com
 【软件版本】V1.1 版权所有，单位使用请先联系授权
 【最近更新】2025-02-25
 【相关信息参考下列地址】
 【网    站】http://www.lqist.cn
 【淘宝店铺】http://longqiu.taobao.com
 --------------------------------------------------------------------------------
 【  IDE  】 keil C251 V5.60
 【Target 】 STC32G/STC8051U/AI8051U 32位模式
 【SYS CLK】 42 MHz使用内部晶振
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/


//下载时, 选择时钟频率 与"config.h"中宏定义保持一致 默认使用42M主频
//实际应用情况中，可能有的外设初始化函数、宏定义不会被用到，为节省8051U空间资源提升效率，可以合理选择将注释掉

#include "include.h"

void System_Init(void);         //上电初始化配置

char txt[64];

/***************************************************************************
* 函     数: void main(void)
* 描     述: 函数入口
* 参     数: None.
* 返     回: None.
* 版     本: V1.0, 2024-11-01
****************************************************************************/
void main(void)
{
	System_Init();
	Global_IRQ_Enable();            //使能全局中断    Global_IRQ_Disable();
	GPIO_LED_Init();
	OLED_Init();  // OLED初始化
	ADC_Init();  // 电感采集初始化
	Timer_EncInit(Timer3|Timer4);  // 编码器初始化
	Motor_Init(Motor_FREQ);  // 电机初始化
	TIM_Init_ms(Timer1, 2);  // 定时器1中断初始化
	TIM_Init_ms(Timer0, 3);  // 定时器1中断初始化
	TIM_Init_ms(Timer11, 3);  // 定时器1中断初始化
	   LQ_SPI_LSM60DSR_Init();     //函数内判断软件还是硬件SPI
	while(1)
	{
 
//		sprintf(txt,"L  :%04d ",Get_ADCResult(4));  // 左横电感
//			OLED_P6x8Str(0,0,txt);
//		sprintf(txt,"L2 :%04d ", Get_ADCResult(5));  // 左竖电感
//			OLED_P6x8Str(0,1,txt);
//		sprintf(txt,"eleOut:%04d ", Get_ADCResult(2));  // 中横电感
//			OLED_P6x8Str(0,2,       txt);
//		sprintf(txt,"R2 :%04d ",Get_ADCResult(1));  // 右竖电感
//			OLED_P6x8Str(0,3,txt);
//		sprintf(txt,"R  :%04d ", Get_ADCResult(0));  // 右横电感
//			OLED_P6x8Str(0,4,txt);

//			

	sprintf(txt,"%03d %03d %04d %03d %03d",left_0,left_1,M,right_0,right_1);		// 显示四路电感归一化值
	OLED_P6x8Str(0,0,(u8*)txt);
		
		sprintf(txt,"M2:%04d",M2);
//	sprintf(txt,"ERV:%.0f k%.3f  ",eleOut,M_K);									// 显示电感偏差值
	OLED_P6x8Str(0,1,(u8*)txt);


	sprintf(txt,"KP:%.5f kd%.5f",Turn_ele[0],Turn_ele[1]);											// 显示弯道PD
	OLED_P6x8Str(0,2,(u8*)txt);
//	sprintf(txt,"an:%.2f wv%.4f",Angle_gyro_z,Weight_V);											//显示积分角度值 与 竖直权重  
//	OLED_P6x8Str(0,2,(u8*)txt);
		sprintf(txt,"Aim :%.0f ", High_Speed);   // 基础目标速度
//			sprintf(txt,"obt:%d",out_ben_turn);
			OLED_P6x8Str(0,3,txt);

		sprintf(txt,"ben :%01d ", flag_ben);   // Benzene ring situation
			OLED_P6x8Str(50,3,txt);
//					sprintf(txt,"Aim :%.1f L :%.1f R :%.1f", High_Speed,Left_High_Speed,Right_High_Speed);   // 基础目标速度
//			OLED_P6x8Str(0,3,txt);
//					sprintf(txt,"Aim_L :%.f ", Left_High_Speed);   // 左轮目标速度
//			OLED_P6x8Str(50,1,txt);
		sprintf(txt,"Aim_R :%.f ", Right_High_Speed);  // 右轮目标速度
			OLED_P6x8Str(50,4,txt);
			
	  sprintf(txt,"fg:%d ",key_flag);     // 按键调参标志位 两路电机PWM
	  OLED_P6x8Str(0,4,(u8*)txt);
			
			sprintf(txt,"%02d%02d",charge_flag,star_flag);  // 左编码器
			OLED_P6x8Str(0,5,txt);
			
			sprintf(txt,"RP:%.01f",Right_MOTOR[0]);
			OLED_P6x8Str(24,5, txt); 	// 显示
			sprintf(txt,"RI:%.02f",Right_MOTOR[1]);
			OLED_P6x8Str(66,5, txt); 	// 显示
			
			sprintf(txt,"LP:%.01f",Left_MOTOR[0]);
			OLED_P6x8Str(0,6, txt); 	// 显示
			sprintf(txt,"LI:%.02f",Left_MOTOR[1]);
			OLED_P6x8Str(56,6, txt); 	// 显示
//		sprintf(txt, "ADV:%03d ", vu);  //ADC8-P00做电源电压 11.0V->1700左右，12.6V->1970左右
//		OLED_P6x8Str(0,6, txt); 	// 显示
			
			
		sprintf(txt,"PL%.f  PR%.f ",LEFT_MOTOR_Duty,RIGHT_MOTOR_Duty);   // 左轮占空比
			OLED_P6x8Str(0,7,txt);

			
		delay_ms(10);
		
	}
}






// 初始化pll时钟,注意使用时需要添加对应的库，详情参考数据手册
void PLL_Init(void)
{
    
	USBCLK &= ~0xf0;
	USBCLK |= 0xd0;     // 允许追频
	CLKSEL |= 0xc0;     // 时钟选择设置，并将PLL时钟作为外设高速时钟
	TFPU_CLKDIV = 0;
	DMAIR = 0x3f;		// 将PLL时钟作为TFPU时钟，0x3e 选择系统时钟为TFPU时钟
}

/***************************************************************************
* 函     数: void main(void)
* 描     述: 函数入口
* 参     数: None.
* 返     回: None.
****************************************************************************/
void System_Init()
{
	WTST = 0;		//设置程序指令延时参数，赋值为0可将CPU执行指令的速度设置为最快
	EAXSFR();		//扩展SFR(XFR)外设寄存器访问使能
	CKCON = 0;      //提高访问XRAM速度
    
    PLL_Init();
    P0_MODE_IN_HIZ(GPIO_Pin_All);       // 上电先将所有IO配置为高阻态，可以防止意外导致管脚烧坏的情况发生
    P1_MODE_IN_HIZ(GPIO_Pin_All);
    P2_MODE_IN_HIZ(GPIO_Pin_All);
    P3_MODE_IN_HIZ(GPIO_Pin_All);
    P4_MODE_IN_HIZ(GPIO_Pin_All);
    P5_MODE_IN_HIZ(GPIO_Pin_All);
    
    PNRST=1;   //P47，即RSTCFG|=1<<4;复位寄存器配置（STC32G/F为P54），防止ISP下载误设为I/O下次需要上电复位才能下载使得复位按键失效

}


