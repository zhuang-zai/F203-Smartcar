#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "LQ_Encoder.h"
#include "LQ_MotorServo.h"
#include "pid.h"
#include "DEF.h"

// 电机数据结构体：集中管理编码器值、PWM 输出、停车标志
// 对外只读，仅可通过 Motor_SetStopFlag 修改 stop_flag
typedef struct {
    int16 encoder_L;
    int16 encoder_R;
    int16 left_pwm;
    int16 right_pwm;
    int16 stop_flag;
} MotorData;

// 返回电机数据的只读指针
const MotorData* Motor_GetData(void);
// 设置停车标志：外部唯一可修改 stop_flag 的接口
void Motor_SetStopFlag(int16 flag);

void Motor_Control(int16 Left_Target_Speed, int16 Right_Target_Speed);

#endif
