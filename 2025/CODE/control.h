#ifndef __CONTROL_H_
#define __CONTROL_H_

#include "include.h"

typedef struct PID
{
	double iError;  // 本次偏差
    double LastError;  // 上次偏差
    double PrevError;  // 上上次偏差
    double KP;
    double KI;
    double KD;
}PID;

#define MOTOR_MAX 3300  // 注意满占空比为3360（42000000/12500）

extern PID Left_MOTOR_PID, Right_MOTOR_PID, Turn_PID_ele;  // 三个结构体变量

extern unsigned char run;  // 电机启停标志位

extern int L, L2, M,M2, R2, R,vu,charge_flag,star_flag,led_flag;  			 		   // 电感值
extern double eleValue, eleSub, eleAdd;  // 差比和

extern double Turn_ele[2];  // 转向环 PD 参数
extern double elemid;		// 转向环目标位置偏差
extern double eleOut;		// 转向环输出值

extern double Left_MOTOR[3];   // 左轮速度环 PID 参数
extern double Right_MOTOR[3];  // 右轮速度环 PID 参数

extern double encoder_L, encoder_R;								// 左右轮编码器数据
extern double LEFT_MOTOR_Duty, RIGHT_MOTOR_Duty; 				// 左右电机占空比
extern double Left_High_Speed, Right_High_Speed, High_Speed;	// 左右轮目标速度、基础目标速度
double Get_Speed_By_Error(double error);
extern double range_protect(double duty,double min,double max);

void Dir_Control();		// 转向控制函数
void Motor_Control();  	// 电机控制函数
void Ultima_Control();  // 综合控制函数
extern double Inductor[4];
extern double PlacePID_Control(PID*sptr, double NowPiont, double SetPoint, double *Turn_PID);  // 位置式 PID
extern double PID_Realize(PID*sptr, double ActualSpeed, double SetSpeed, double *MOTOR_PID);   // 增量式 PID
extern int16 left_0,left_1,right_0,right_1;
extern void key_scan(void);
extern int key_flag;
extern int flag_ben;
extern int out_ben;
extern int out_ben_turn;
extern int time_ben;
extern void Angle_calculate();
double error_to_output(double error);
extern double Calculate_Weight_Mid(double M);
extern void Calculate_Error();
extern double Calculate_Vertical();
 extern   double Weight_H;  // 水平权重
 extern   double Weight_V;  // 竖直权重
 extern int Median_Average_Filter(int* arr, int times);
 extern int L_md[4],L2_md[4],M_md[4],R2_md[4],R_md[4];
extern void adc_filter(void);
extern double M_K;
extern double Angle_gyro_z; 
double angle_pid(double target, double current);
extern double current_angle;     // 当前累计旋转角度
#endif
