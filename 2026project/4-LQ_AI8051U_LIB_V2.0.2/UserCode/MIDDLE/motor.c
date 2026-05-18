#include "motor.h"
#include "stdio.h"
int16 _encoder_L;
int16 _encoder_R;
int16 left_pwm;
int16 right_pwm;

int16 vofa_target_speed = 0;
int16 vofa_current_speed = 0;
int16 vofa_out_pwm = 0;

/**
 * @brief 电机速度控制函数：输入目标速度，读取当前速度，通过PID计算占据比，控制电机转速
 * @param Left_Target_Speed 左轮目标速度 Right_Target_Speed 右轮目标速度
 * @param 无返回值，但可能会出错
 */
void Motor_Control(int16 Left_Target_Speed, int16 Right_Target_Speed)
{
		_encoder_L = -1 * Read_Encoder(1);//读取编码器数值(真实速度) ,左编码器前进为负值因此前面加上负号
		_encoder_R = Read_Encoder(2);//读取编码器数值(真实速度)

	//速度环占据比，分别用PID_LEFT和PID_RIGHT索引来实现左右轮控制
	left_pwm = PID_Calculate_ByIndex(PID_LEFT, _encoder_L, Left_Target_Speed);
	right_pwm = PID_Calculate_ByIndex(PID_RIGHT, _encoder_R, Right_Target_Speed);
	//Motor_Ctrl(600,600);
	/*
	当3000占空比时对应编码器输出为1100，则认为左右轮最大转速为1100
	*/
	Motor_Ctrl(right_pwm, left_pwm); //输出需要自己确保正确
	//printf("%d,%d,%d\n", Left_Target_Speed, _encoder_L, left_pwm);
	
	vofa_target_speed = Left_Target_Speed;
  vofa_current_speed = _encoder_L;
  vofa_out_pwm = left_pwm;
}
