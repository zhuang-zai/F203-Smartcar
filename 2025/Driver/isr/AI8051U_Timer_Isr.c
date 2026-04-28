/*---------------------------------------------------------------------*/
/* ------------------- Web: www.STCAI.com -----------------------------*/
/*---------------------------------------------------------------------*/

#include "AI8051U_Timer.h"
#include "include.h"

//========================================================================
// 函数: Timer0_ISR_Handler
// 描述: Timer0中断函数.
// 参数: none.
// 返回: none.
// 版本: V1.0, 2024-11-23
//========================================================================
int last_M=0;
int ben_panduan=0;
u8 times_count =0;
void Timer0_ISR_Handler (void) interrupt TMR0_VECTOR		//进中断时已经清除标志
{
	// TODO: 在此处添加用户代码

//    times_count++;
//    if(times_count>20)   //200ms
    {
			


//	L=(Get_ADCResult(4)+Get_ADCResult(4))/2;
//	L2=(Get_ADCResult(5)+Get_ADCResult(5))/2;
//	R2=(Get_ADCResult(1)+Get_ADCResult(1))/2;
//	R=(Get_ADCResult(0)+Get_ADCResult(0))/2;
//	M = Get_ADCResult(2);   // 中间电感
//			
//			
//			
	Inductor[0]=L;
	Inductor[1]=L2;
	Inductor[2]=R2;
	Inductor[3]=R;
			
			
	left_0=(float)(Inductor[0]-10)*100/(1845-10);
	left_1=(float)(Inductor[1]-10)*100/(1280-10);
	right_0=(float)(Inductor[2]-10)*100/(1280-10);
	right_1=(float)(Inductor[3]-10)*100/(1845-10);

			
			
if(run==1){
		if(M>1900&&flag_ben==0)        //苯环  原先M >2300
		{
			current_angle=Angle_gyro_z*1.0;
			if(L>R)
			flag_ben=1;
			else if(L<R)
				flag_ben=2;		
		}
		else if(M<900&&flag_ben==1)    //左进环
		flag_ben=3;    //进环标志
		
		else if(M<1000&&flag_ben==2)   //右进环
			flag_ben=4;
		
		else if(flag_ben==3&&M>1800){          //左出环标志
			flag_ben=0;        //出环标志
//			current_angle=Angle_gyro_z*1.0;
		}
//		else if(out_ben == 1 && left_0 > 100) out_ben_turn = 1,out_ben = 0;
		else 		if(flag_ben==4&&M>1800){          //右出环标志
			flag_ben=0;        //出环标志
//			current_angle=Angle_gyro_z*1.0;
		}
		
		
		
		
//		if(flag_ben==6&&M<1500&&Angle_gyro_z>current_angle+60.0){
//				flag_ben=0;       //出环
//			time_ben=0;
//		}
//		else 		if(flag_ben==5&&M<1500&&Angle_gyro_z>current_angle-60.0){
//				flag_ben=0;       //出环
//			time_ben=0;
//		}
}
			led_flag++;
			if(led_flag%50==0)
        LED_Ctrl(LED0,RVS);
			else LED_Ctrl(LED0,OFF);
    }
}

//========================================================================
// 函数: Timer1_ISR_Handler
// 描述: Timer1中断函数.
// 参数: none.
// 返回: none.
// 版本: V1.0, 2020-09-23
//========================================================================
void Timer1_ISR_Handler (void) interrupt TMR1_VECTOR		//进中断时已经清除标志
{
	// TODO: 在此处添加用户代码
//	LED_Ctrl(LED0,RVS);
	
	Ultima_Control();
}

//========================================================================
// 函数: Timer2_ISR_Handler
// 描述: Timer2中断函数.
// 参数: none.
// 返回: none.
// 版本: V1.0, 2020-09-23
//========================================================================
void Timer2_ISR_Handler (void) interrupt TMR2_VECTOR		//进中断时已经清除标志
{
	// TODO: 在此处添加用户代码
//	LED_Ctrl(LED0,RVS);

}

//========================================================================
// 函数: Timer3_ISR_Handler
// 描述: Timer3中断函数.
// 参数: none.
// 返回: none.
// 版本: V1.0, 2020-09-23
//========================================================================
void Timer3_ISR_Handler (void) interrupt TMR3_VECTOR		//进中断时已经清除标志
{
	// TODO: 在此处添加用户代码
//	LED_Ctrl(LED0,RVS);

}

//========================================================================
// 函数: Timer4_ISR_Handler
// 描述: Timer4中断函数.
// 参数: none.
// 返回: none.
// 版本: V1.0, 2020-09-23
//========================================================================
void Timer4_ISR_Handler (void) interrupt TMR4_VECTOR		//进中断时已经清除标志
{
	// TODO: 在此处添加用户代码
//	LED_Ctrl(LED1,RVS);
}


//========================================================================
// 函数: Timer4_ISR_Handler
// 描述: Timer4中断函数.
// 参数: none.
// 返回: none.
// 版本: V1.0, 2020-09-23
//========================================================================
void Timer11_ISR_Handler (void) interrupt TMR11_VECTOR		//进中断时已经清除标志
{
	// TODO: 在此处添加用户代码
//	LED_Ctrl(LED0,RVS);
//    LED_Ctrl(Beep0,RVS);		
	key_scan();

}


