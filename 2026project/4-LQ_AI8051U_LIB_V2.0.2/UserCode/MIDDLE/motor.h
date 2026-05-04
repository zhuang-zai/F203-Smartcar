#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "LQ_Encoder.h"
#include "LQ_MotorServo.h"
#include "pid.h"
#include "DEF.h"

void Motor_Control(int16 Left_Target_Speed, int16 Right_Target_Speed);

#endif