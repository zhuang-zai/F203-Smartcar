#include "motor.h"
#include "stdio.h"

// 电机数据全局结构体：包含编码器值、PWM输出、停车标志，集中管理电机状态
static MotorData motor = {0};

/*pwm输出限幅*/
// 正转限制在约 85%，防止占空比过大导致失控
static int16 max_forward_pwm = (int16)(3200 * 0.8);  // 正转限制在约 85%
// 反转限制在约 -40%，刹车力度够用即可
static int16 max_reverse_pwm = (int16)(-3200 * 0.4) ; // 反转限制在约 -40% (刹车力度够用就行)
/**
 * @brief 电机速度控制函数：输入目标速度，读取当前速度，通过PID计算占据比，控制电机转速
 * @param Left_Target_Speed 左轮目标速度 Right_Target_Speed 右轮目标速度
 * @param 无返回值，但可能会出错
 */
void Motor_Control(int16 Left_Target_Speed, int16 Right_Target_Speed)
{
    // stop_flag 为 0 时正常控制，为 1 时停止输出（电感异常时由 Calculate_Deviation 触发）
    if (motor.stop_flag == 0)
    {
        motor.encoder_L = -1 * Read_Encoder(1);//读取编码器数值(真实速度) ,左编码器前进为负值因此前面加上负号
        motor.encoder_R = Read_Encoder(2);//读取编码器数值(真实速度)

        //速度环占据比，分别用PID_LEFT和PID_RIGHT索引来实现左右轮控制
        motor.left_pwm = PID_Calculate_ByIndex(PID_LEFT, motor.encoder_L, Left_Target_Speed);
        motor.right_pwm = PID_Calculate_ByIndex(PID_RIGHT, motor.encoder_R, Right_Target_Speed);
            
        // PWM 输出限幅：正向不超过 max_forward_pwm，反向不超过 max_reverse_pwm
        motor.left_pwm = motor.left_pwm > max_forward_pwm ? max_forward_pwm : motor.left_pwm;
        motor.left_pwm = motor.left_pwm < max_reverse_pwm ? max_reverse_pwm : motor.left_pwm;
        
        motor.right_pwm = motor.right_pwm > max_forward_pwm ? max_forward_pwm : motor.right_pwm;
        motor.right_pwm = motor.right_pwm < max_reverse_pwm ? max_reverse_pwm : motor.right_pwm;
    }
    else	
    {
        // 停车状态：PWM 输出清零
        motor.left_pwm = 0;
        motor.right_pwm = 0;
    }
	//Motor_Ctrl(600,600);
	/*
	当3000占空比时对应编码器输出为1100，则认为左右轮最大转速为1100
	*/
	Motor_Ctrl(motor.right_pwm, motor.left_pwm); //输出需要自己确保正确
	//printf("%d,%d,%d\n", Left_Target_Speed, motor.encoder_L, motor.left_pwm);
}

// 返回电机数据的只读指针：外部通过 const MotorData* 访问编码器、PWM、stop_flag
const MotorData* Motor_GetData(void)
{
    return &motor;
}

// 设置停车标志：外部唯一可修改 motor.stop_flag 的接口
// flag=0 正常运行，flag=1 停止电机输出
void Motor_SetStopFlag(int16 flag)
{
    motor.stop_flag = flag;
}
