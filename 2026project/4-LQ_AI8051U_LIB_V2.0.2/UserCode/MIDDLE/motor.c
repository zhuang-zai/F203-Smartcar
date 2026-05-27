#include "motor.h"
#include "stdio.h"
int16 _encoder_L;
int16 _encoder_R;
int16 left_pwm;
int16 right_pwm;

int16 vofa_target_speed = 0;
int16 vofa_current_speed = 0;
int16 vofa_out_pwm = 0;

int16 stop_flag = 1;

/*pwm输出限幅*/
int16 max_forward_pwm = (int16)(3200 * 0.8);  // 正转限制在约 85%
int16 max_reverse_pwm = (int16)(-3200 * 0.4) ; // 反转限制在约 -40% (刹车力度够用就行)
/**
 * @brief 电机速度控制函数：输入目标速度，读取当前速度，通过PID计算占据比，控制电机转速
 * @param Left_Target_Speed 左轮目标速度 Right_Target_Speed 右轮目标速度
 * @param 无返回值，但可能会出错
 */
void Motor_Control(int16 Left_Target_Speed, int16 Right_Target_Speed)
{
//		static uint8 Motor_stop_Time = 0; // 堵转计时器
//    static int16 left_pwm_last = 0;   // 记录上一次的 PWM (用于滤波)
//    static int16 right_pwm_last = 0;
		_encoder_L = -1 * Read_Encoder(1);//读取编码器数值(真实速度) ,左编码器前进为负值因此前面加上负号
		_encoder_R = Read_Encoder(2);//读取编码器数值(真实速度)
	
		/*堵转检测机制 (撞墙保护)*/
//		if ((Left_Target_Speed != 0 || Right_Target_Speed != 0) && 
//         (abs(_encoder_L) < 5 || abs(_encoder_R) < 5) && 
//         stop_flag == 0)
//		{
//				Motor_stop_Time++;
//        if (Motor_stop_Time >= 50) // 假设2ms周期，50次就是持续100ms堵转
//        {
//            stop_flag = 1;         // 触发急停！
//        }
//		}
//		else 
//		{
//				Motor_stop_Time = 0;       // 恢复正常，清零计数器
//		}

	//速度环占据比，分别用PID_LEFT和PID_RIGHT索引来实现左右轮控制
	left_pwm = PID_Calculate_ByIndex(PID_LEFT, _encoder_L, Left_Target_Speed);
	right_pwm = PID_Calculate_ByIndex(PID_RIGHT, _encoder_R, Right_Target_Speed);
		
			/*输出低通滤波  防止电流突变*/
//		left_pwm = (int16)(left_pwm * 0.8f + left_pwm_last * 0.2f);
//    right_pwm = (int16)(right_pwm * 0.8f + right_pwm_last * 0.2f);
//    left_pwm_last = left_pwm;
//    right_pwm_last = right_pwm;
		
		left_pwm = left_pwm > max_forward_pwm ? max_forward_pwm : left_pwm;
    left_pwm = left_pwm < max_reverse_pwm ? max_reverse_pwm : left_pwm;
    
    right_pwm = right_pwm > max_forward_pwm ? max_forward_pwm : right_pwm;
    right_pwm = right_pwm < max_reverse_pwm ? max_reverse_pwm : right_pwm;
		
		
		if (stop_flag == 1)
    {
        left_pwm = 0;
        right_pwm = 0;
    }
	//Motor_Ctrl(600,600);
	/*
	当3000占空比时对应编码器输出为1100，则认为左右轮最大转速为1100
	*/
	Motor_Ctrl(-right_pwm, -left_pwm); //输出需要自己确保正确
	//printf("%d,%d,%d\n", Left_Target_Speed, _encoder_L, left_pwm);
	
	vofa_target_speed = Left_Target_Speed;
  vofa_current_speed = _encoder_L;
  vofa_out_pwm = left_pwm;
}
