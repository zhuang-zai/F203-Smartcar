#ifndef __CHASSIS_H_
#define __CHASSIS_H_

#include "pid.h"

// 底盘数据结构体：集中管理目标速度、循迹偏差、左右轮速度、方向环输出
// 对外只读，通过 Chassis_GetData() 返回 const 指针
typedef struct
{
    int16 target_speed;
    int current_deviation;
    int16 left_speed;
    int16 right_speed;
    int16 direction_output;
} Chassis_TypeDef;

// 返回底盘数据的只读指针
const Chassis_TypeDef* Chassis_GetData(void);

void Chassis_Init(void);
void Chassis_Control(void);
int Calculate_Deviation(void);

#endif
