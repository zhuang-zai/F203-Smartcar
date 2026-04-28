#include "control.h"
#include <math.h>

#define DEGREE_PER_PULSE 10

PID Left_MOTOR_PID, Right_MOTOR_PID, Turn_PID_ele;  // 三个结构体变量

unsigned char run = 0;  // 电机启停标志位

int L, L2, M,M2, R2, R,vu,led_flag;				// 电感值
double eleValue, eleSub, eleAdd;	// 差比和
int16 left_0,left_1,right_0,right_1;
double Turn_ele[2]={0.00095, 0.00135};	// 转向环 PD 参数  原先t_e[0] = 0.00095,t_e[1] = 0.00135
double elemid = 0;					// 转向环目标位置偏差
double eleOut = 0;					// 转向环输出值
int z=0;
int L_md[4],L2_md[4],M_md[4],R2_md[4],R_md[4];
double k1;
int key_flag=1,star_flag=0;

double Inductor[4];      //电感值

int charge_flag=0;

double kp = 1.0f;   
double ki = 0.0f;   
double kd = 0.15;    

double last_encoder_L = 0;
double last_encoder_R = 0;
double current_angle;     // 当前累计旋转角度
double target_angle = 90.0f;     // 目标角度（单位：度，可修改）

double delta_L ;
double delta_R ;
 
double angle_delta;




double Left_MOTOR[3] = {23,4.1, 0};   // 左轮速度环 PID 参数
double Right_MOTOR[3] = {25, 4, 0};  // 右轮速度环 PID 参数

double encoder_L, encoder_R;							// 左右轮编码器数据
double LEFT_MOTOR_Duty, RIGHT_MOTOR_Duty;				// 左右电机占空比
double Left_High_Speed, Right_High_Speed, High_Speed=110;	// 左右轮目标速度、基础目标速度

double M_K;
int time_ben=0;
int flag_ben=0;
int out_ben = 0;
int out_ben_turn = 0;
int chu=0;
signed short acc_X, acc_Y, acc_Z;
signed short gyro_X, gyro_Y, gyro_Z;
float Gyroz;  // 偏航角速度
double Angle_gyro_z;  // 积分角度
 
    double Weight_H = 0;  // 水平权重
    double Weight_V = 0;  // 竖直权重

/**
 * @brief 限幅保护函数
 */


void key_scan(void){

      if (!KEY_Read(KEY0))		// K0按键调参 直道 弯道PID 速度x
        {
            switch(key_flag)
            {
                case 1: High_Speed= High_Speed+10.0; break;
                case 2:  Turn_ele[0]+=0.0001;       break;
								case 3:  Turn_ele[1]+=0.0001;     break;
							  case 4:  	Left_MOTOR[0]+=0.1;			break;
							  case 5:		Left_MOTOR[1]+=0.01;    break;
								case 6:		Right_MOTOR[0]+=0.1;     break;
                case 7:   Right_MOTOR[1]+=0.01;      break;
            }
            delay_ms(200);   
        }
        if (!KEY_Read(KEY1))  // K1按键调参 直道 弯道PID 速度
        {
            switch(key_flag)
            {
                case 1: High_Speed= High_Speed-10.0;   break;
                case 2:  Turn_ele[0]-=0.0001;       break;
								case 3:  Turn_ele[1]-=0.0001;     break;
                case 4:  	Left_MOTOR[0]-=0.1;			break;
							  case 5:		Left_MOTOR[1]-=0.01; break;
								case 6:		Right_MOTOR[0]-=0.1;     break;
                case 7:   Right_MOTOR[1]-=0.01;      break;
            }
            delay_ms(200);
        }
        if(!KEY_Read(KEY2))  // K2按键选择需要调节参数
        {
					key_flag = (key_flag+1)%8;
					if(key_flag==0) key_flag = 1;
//            switch(key_flag)
//            {
//                case 1: key_flag = 2; break;
//                case 2: key_flag = 3; break;
//                case 3: key_flag = 4; break;
//                case 4: key_flag = 1; break;
//                case 5: key_flag = 6; break;
//								case 6: key_flag = 7; break;
//								case 7: key_flag = 8; break;
//								case 8: key_flag = 1;break;
//            }
            delay_ms(200);
        }



}



double range_protect(double duty, double min, double max)
{
    return duty >= max ? max : (duty <= min ? min : duty);
}


/**
 * @brief 方向环控制函数，算出最终左右轮目标速度
 */
void Dir_Control()
{
	double Angle;  // 差速系数
	
	/* 采集各通道电感值 */
	L = Get_ADCResult(4);   // 左横电感
	L2 = Get_ADCResult(5);  // 左竖电感
	M = Get_ADCResult(2);   // 中间电感
	M2 = Get_ADCResult(3);  //新加  中竖电感
	R2 = Get_ADCResult(1);  // 右竖电感
	R = Get_ADCResult(0);   // 右横电感
	
	M_K=Calculate_Vertical();
	
  LQ_SPI_LSM60DSR_Read(&acc_X, &acc_Y, &acc_Z, &gyro_X, &gyro_Y, &gyro_Z);
	    Weight_H = Calculate_Weight_Mid(M);  // M ↑，水平循迹比重 ↑
    Weight_V = 1.0000 - Weight_H;



	if(M<25)M=25;
	
	/* 差比和计算赛道偏差 */

//	eleSub = Weight_H * (L - R) + Weight_V * (L2 - R2);    速度快  但是飘
//	eleAdd = (1-M_K+0.0001) * (Weight_H * (L + R) + Weight_V * (L2 + R2) + M);
	eleSub = L - R +L2 - R2; //慢但是准
	eleAdd = L + R + L2 + R2 + M;
	eleValue = (eleSub / eleAdd) * 300000.0f;	// 差比和计算：赛道偏差
	
	Angle_calculate();
	
	if(flag_ben==0||flag_ben==3||flag_ben==4){
	eleOut = PlacePID_Control(&Turn_PID_ele, elemid, eleValue, Turn_ele);  // 调用位置式 PID
	}
//     -----------------------------苯环只写了右入的  逻辑相反
		else if(flag_ben==2){

			eleOut = -1*angle_pid(current_angle-75.0,Angle_gyro_z);
			eleOut = range_protect(eleOut, -80, 80);
		}
//		else if(flag_ben==6){

//	eleOut = -1*angle_pid(current_angle-65.0,Angle_gyro_z);
//			eleOut = range_protect(eleOut, -80, 80);
//		}
		
		 
		 
		else if(flag_ben==1){

	eleOut = -1*angle_pid(current_angle+80 ,Angle_gyro_z);
	eleOut = range_protect(eleOut, -82, 82);
		}
//		else if(flag_ben == 0 && out_ben == 1)  //加入出苯环判断语句
//		{
//			eleOut = -1*angle_pid(current_angle+60,Angle_gyro_z);
//			eleOut = range_protect(eleOut, -80, 80);
//			out_ben = 0;
//		}
//		else if(flag_ben==5){

//	eleOut = -1*angle_pid(current_angle+65,Angle_gyro_z);
//			eleOut = range_protect(eleOut, -80, 80);
//		}
	eleOut = range_protect(eleOut, -85, 85);  // 限幅保护，防止差速太大
		
		/*Battery condition*/
vu=Get_ADCResult(8)*(12.6-11)/(1970-1700)+1.1+11-(12.6-11)/(1970-1700)*1700;  //??
	if(vu*2>15)
		charge_flag=1;
	
	else if(vu*2<13)
		charge_flag=0;
	
	else if(vu*2>13&&vu*2<15)charge_flag=2;

	else if (star_flag==0&&(charge_flag==2||charge_flag==0))
		Motor_Ctrl(0,0);
		
	/*Start when the battery is fully charged*/
	if(vu*2>15)
	{
		star_flag=1;
	}
	
	
			/* 冲出赛道保护 */
	if(L<15 && L2<15 && R2<15 && R<15)
	{
		run=0;
	}
	else if(L>15 && L2>15 && R2>15 && R>15&&star_flag==1)
	{
		run=1;
	}
	
	/* 速度设置 & 差速计算 */
	if(run==1)  // 在赛道内
	{
//					if(eleOut>60)
//							High_Speed = 115;
//					else if (eleOut<60&&eleOut>30)
//						High_Speed = 125;
//					else if (eleOut<30)
//						High_Speed = 130;		
					if(eleOut>60)
							High_Speed = 60;  //90
					else if (eleOut<60&&eleOut>30)
						High_Speed = 65;  //95
					else if (eleOut<30)
						High_Speed = 75;			//105
					else if (eleOut<10)
						High_Speed = 90;  //120
					
					
					/*Enter the benzene ring*/
					if(M>1900)
					{
						High_Speed = 50;
					}
//					
//			double abs_error = fabs(eleOut);
//    High_Speed = 114.66 * exp(-0.04 * abs_error) + 106.84;
//					
//    if(High_Speed > 135) High_Speed = 135;
//    if(High_Speed < 115) High_Speed = 115;
//		.3333333333333333330
//				if(M>2300&&flag_ben==0)        //苯环
//			
//		{

//			current_angle=Angle_gyro_z*1.0;
//			if(L>R)
//			flag_ben=1;
//			else if(L<R)
//				flag_ben=2;
//					
//		}
//		else if(M<1000&&flag_ben==1&&Angle_gyro_z>current_angle-70.0)    //左进环
//		flag_ben=3;    //进环标志
//		
//		else if(M<1000&&flag_ben==2&&Angle_gyro_z>current_angle+70.0)   //右进环
//			flag_ben=4;
//		
//		if(flag_ben==3&&M>2000){          //左出环标志
//			flag_ben=0;        //出环标志
////			current_angle=Angle_gyro_z*1.0;
//		}
//		else 		if(flag_ben==4&&M>2000){          //右出环标志
//			flag_ben=0;        //出环标志
////			current_angle=Angle_gyro_z*1.0;
//		}
//		
//		
//		
		
		
		
		
		
  // 在这里修改基础速度
	//	High_Speed=error_to_output(eleOut);

		
		if(eleOut >= 0)  // 需要左转
		{
			Angle = (eleOut);
			Left_High_Speed = High_Speed  - Angle*0.65 +gyro_Z*0.001;  //0.001
			Right_High_Speed = High_Speed + Angle*0.55*0.7 -gyro_Z*0.001;  // 外轮加得少一点
		}
		else  // 需要右转
		{
			Angle = (-eleOut) ;
			Left_High_Speed = High_Speed  + Angle*0.55*0.7-gyro_Z*0.001;  // 外轮加得少一点
			Right_High_Speed = High_Speed  - Angle*0.65+gyro_Z*0.0015;
		}

	}
	else if(run==0)  // 出赛道 进入保护
	{
		High_Speed = 0;
		Left_High_Speed = 0;
		Right_High_Speed = 0;
	}
}


/**
 * @brief 电机控制函数，实现左右轮速度闭环控制
 */
void Motor_Control()
{
	// 编码器数据在屏幕上已经显示，一定要核对确认：①左右不要搞反  ②正负不要搞反
			encoder_L = -Read_Encoder(Enc_2);  // 左编码器
			encoder_R = Read_Encoder(Enc_1);   // 右编码器
	
	// 下面有 A、B 两种占空比赋值方式，A 用于测试电机左右和正反转是否对应，B 可实现速度闭环
	
	// A：人为给定占空比，用于测试：①左右轮不要搞反  ②正反转不要搞反
//	LEFT_MOTOR_Duty = 2000;  	// 2000 效果：左轮正转，速度较快
//	RIGHT_MOTOR_Duty = -1200;	//  -1200 效果：右轮反转，速度较慢
	
	// B：闭环计算占空比，在确认编码器和电机对应无误后，可实现闭环
	LEFT_MOTOR_Duty += PID_Realize(&Left_MOTOR_PID, encoder_L, Left_High_Speed, Left_MOTOR);
	RIGHT_MOTOR_Duty += PID_Realize(&Right_MOTOR_PID, encoder_R, Right_High_Speed, Right_MOTOR);


	if(LEFT_MOTOR_Duty>0)
	{
			LEFT_MOTOR_Duty = range_protect(LEFT_MOTOR_Duty, 0, MOTOR_MAX);
	}
	else
	{
			LEFT_MOTOR_Duty = range_protect(LEFT_MOTOR_Duty, -MOTOR_MAX, 0);
	}

	if(RIGHT_MOTOR_Duty>0)
	{
			RIGHT_MOTOR_Duty = range_protect(RIGHT_MOTOR_Duty, 0, MOTOR_MAX);
	}
	else
	{
			RIGHT_MOTOR_Duty = range_protect(RIGHT_MOTOR_Duty, -MOTOR_MAX, 0);
	}
Motor_Ctrl(-1*(int16)RIGHT_MOTOR_Duty, -1*(int16)LEFT_MOTOR_Duty);
}


/**
 * @brief 综合控制函数，在中断中调用
 */

void Ultima_Control()
{

	Dir_Control();  // 转向控制
	Motor_Control();  //电机控制

}





/**
 * @brief 位置式 PID（用于转向控制）
 */
double PlacePID_Control(PID*sptr, double NowPiont, double SetPoint, double *Turn_PID)
{
	double Output;  // 本次输出
	
	sptr->KP = *Turn_PID;  // 参数赋值
	sptr->KD = *(Turn_PID+1);
	
	sptr->iError = SetPoint - NowPiont;  // 当前误差 = 目标值 - 实际值
	
	Output = sptr->KP * sptr->iError  // 比例项
		   + sptr->KD * (sptr->iError - sptr->LastError)  // 微分项
			;
	
	sptr->LastError = sptr->iError;  // 更新误差
	
	return Output;
}


/**
 * @brief 增量式 PID（用于左右轮电机闭环控制）
 */
double PID_Realize(PID* sptr, double ActualSpeed, double SetSpeed, double *MOTOR_PID)
{
    double Increase;        // 单次PID输出
    double iTermLimited;    // 限幅后的积分项

    // 参数赋值
    sptr->KP = *MOTOR_PID;
    sptr->KI = *(MOTOR_PID + 1);
    sptr->KD = *(MOTOR_PID + 2);

    // 计算当前误差
    sptr->iError = SetSpeed - ActualSpeed;

    // 积分项限幅：限制 KI * iError 在 [-900, +900]
    iTermLimited = sptr->KI * sptr->iError;
    if (iTermLimited > 1000) iTermLimited = 1000;
    else if (iTermLimited < -1000) iTermLimited = -1000;

    // PID增量计算
    Increase = sptr->KP * (sptr->iError - sptr->LastError)
             + iTermLimited
             + sptr->KD * (sptr->iError - 2 * sptr->LastError + sptr->PrevError);

    // 更新误差
    sptr->PrevError = sptr->LastError;
    sptr->LastError = sptr->iError;

    return Increase;
}




// 函数：动态权重，分配水平循迹和竖直循迹占比
double Calculate_Weight_Mid(double M)
{
    double k = 0;
 
    if(M >= 1400)  // M >= 80，权重为 100%
    {
        k = 100.0;
    }
    else if (M >= 400&& M < 1400)  // 线性权重
    {
        k = 0.1*M -60;  // y=kx+b：根据 (800,0) 和 (2200,100) 解二元一次方程组
    }
    else  // M < 40，权重为 0%
    {
        k = 0.0;
    }
    
    k *= 0.01;  // 将范围控制在：0.0 ~ 1.0
    return k >= 1.0 ? 1.0 : (k <= 0.0 ? 0.0 : k);  // 将 k 限幅保护在 0.0 ~ 1.0 之间
}




void Calculate_Error()
{
    Weight_H = Calculate_Weight_Mid(M);  // M ↑，水平循迹比重 ↑
    Weight_V = 1.0 - Weight_H;
}




double Calculate_Vertical() 
{	
	double k = 0;
	double sum = 4680.0;  // 两个竖直电感最大值的和
	
	k = fabs(L2 - R2) / sum;  // 浮点型绝对值使用 fabs()
 
	// 分母不能为 0，所以最小保护为 0.1
	return k >= 1.0 ? 1.0 : (k <= 0.1 ? 0.1 : k);  // 将 k 限幅保护在 0.1 ~ 1.0 之间
}




void Angle_calculate()
{     
	Gyroz=(double)gyro_Z;
    if(Gyroz<32768)Gyroz = - (Gyroz / 16.4);
    if(Gyroz>=32768)Gyroz = + (65535 - Gyroz) / 16.4;
 
    Angle_gyro_z += Gyroz * 0.005;  // 根据中断周期进行微调
}


/**
 * 中位值平均滤波函数
 * @param arr 输入数据数组
 * @param times 数据采样次数
 * @return 滤波后的平均值
 */
int Median_Average_Filter(int* arr, int times)
{
    int min = arr[0], max = arr[0], sum = 0;
    int i = 0;
    
    // 遍历数组，找出最大值、最小值并求和
    for (i = 0; i < times; i++)
    {
        if (arr[i] < min) min = arr[i];
        if (arr[i] > max) max = arr[i];
        sum += arr[i];
    }
 
    // 计算去掉最大值和最小值后的平均值
    return (sum - min - max) / (times - 2);
}
void adc_filter(void){

	int z=0;
	
	
		for( z=0;z<4;z++){
	  L_md[z]=Get_ADCResult(4);
	  L2_md[z]=Get_ADCResult(5);
		M_md[z]=Get_ADCResult(2);
	  R2_md[z]=Get_ADCResult(1);
	  R_md[z]=Get_ADCResult(0);
	}
	
	
	L =Median_Average_Filter(L_md,4);
	L2 =Median_Average_Filter(L2_md,4);
	M =Median_Average_Filter(M_md,4);
	R2 =Median_Average_Filter(R2_md,4);
	R =Median_Average_Filter(R_md,4);
	
}


double angle_pid(double target, double current)
{
		double outputa=0;
		double last_error = 0.0f;
		double integral = 0.0f;
    double derivative;
    double error = 0.0;
		error = target - current;
    integral += error;

		derivative = error - last_error;
   outputa = kp * error + ki * integral + kd * derivative;

    // 限幅处理
    if (outputa > 80) outputa = 80;
    else if (outputa < -80) outputa = -80;

    last_error = error;
    return outputa;
}



double Get_Speed_By_Error(double error)
{
		double A = 106.38;
    double k = 0.05;
    double B = 106.27;
    
    double abs_error = fabs(error);
    double speedd = 106.38 * exp(-0.05 * abs_error) + 106.27;

    // 可选：加个限幅保护
    if(speedd > 130) speedd = 130;
    if(speedd < 115) speedd = 115;

    return speedd;
}

