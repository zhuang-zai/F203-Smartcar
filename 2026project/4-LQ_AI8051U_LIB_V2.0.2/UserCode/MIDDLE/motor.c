#include "motor.h"

/**
 * @brief 电机速度控制函数：输入目标速度，读取当前速度，通过PID计算占据比，控制电机转速
 * @param Left_Target_Speed 左轮目标速度 Right_Target_Speed 右轮目标速度
 * @param 无返回值，但可能会出错
 */
void Motor_Control(int16 Left_Target_Speed, int16 Right_Target_Speed)
{
 int16 _encoder_L = Read_Encoder(1);//读取编码器数值(真实速度)
 int16 _encoder_R = -1 * Read_Encoder(2);//读取编码器数值(真实速度)

 //速度环占据比，分别用PID_LEFT和PID_RIGHT索引来实现左右轮控制
 Left_Target_Speed = PID_Calculate_ByIndex(PID_LEFT, _encoder_L, Left_Target_Speed);
 Right_Target_Speed = PID_Calculate_ByIndex(PID_RIGHT, _encoder_R, Right_Target_Speed);

 Motor_Ctrl(Right_Target_Speed, Left_Target_Speed); //输出需要自己确保正确
}
