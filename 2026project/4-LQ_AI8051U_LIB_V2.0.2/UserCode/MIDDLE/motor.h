#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "LQ_Encoder.h"
#include "LQ_MotorServo.h"
#include "pid.h"
#include "DEF.h"

extern int16 _encoder_L;
extern int16 _encoder_R;
extern int16 left_pwm;
extern int16 right_pwm;
extern int16 vofa_target_speed;
extern int16 vofa_current_speed;
extern int16 vofa_out_pwm;
void Motor_Control(int16 Left_Target_Speed, int16 Right_Target_Speed);

#endif