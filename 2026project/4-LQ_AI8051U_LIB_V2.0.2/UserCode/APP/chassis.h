#ifndef __CHASSIS_H_
#define __CHASSIS_H_

#include "pid.h"

// 底盘控制结构体
typedef struct
{
    int16 target_speed;        // 目标基础速度
    int current_deviation;    // 当前赛道偏离程度（外环输出）
    int16 left_speed;         // 左轮目标速度
    int16 right_speed;        // 右轮目标速度
} Chassis_TypeDef;

// 函数声明
void Chassis_Init(void);
void Chassis_Control(void);
int Calculate_Deviation(void);  // 差比和算法计算偏离值

#endif