#include "chassis.h"
#include "inductance.h"
#include "motor.h"
#include "LQ_Encoder.h"

// 底盘数据全局结构体：集中管理目标速度、循迹偏差、左右轮速度、方向环输出
static Chassis_TypeDef chassis;

/**
 * @brief 底盘初始化
 * 初始化底盘结构体中的速度和目标偏差变量
 */
void Chassis_Init(void)
{
	chassis.target_speed = 150;       // 初始化目标速度为150   max : 1100
    chassis.current_deviation = 0;  // 初始化当前偏差为0
    chassis.left_speed = 0;         // 初始化左轮速度为0
    chassis.right_speed = 0;        // 初始化右轮速度为0
    chassis.direction_output = 0;
}

/**
 * @brief 计算循迹偏差
 * 使用差比和算法计算车辆偏离中心线的程度
 * 公式：( (左感+左感) - (右感+右感) ) / (总电感 + 补偿值) * 100
 * @return int 归一化后的偏差值，范围 -100 到 100
 */
int Calculate_Deviation(void) 
{
    // 获取电感ADC数组指针 (假设顺序为: 左外, 左内, 中, 右内, 右外)
    int* adc_values = GetInductance();
    
    // 计算左右两侧电感值的差值 (分子)，反映偏向哪一侧
    int eleSub = (adc_values[0] + adc_values[1]) - (adc_values[3] + adc_values[4]);
    
    // 计算所有电感值的总和 (分母)，+10是为了防止分母为0导致除零错误
    // 使用"差比和"可以消除由于速度变化导致的磁场强度整体变化影响
    int eleAdd = adc_values[0] + adc_values[1] + adc_values[2] + adc_values[3] + adc_values[4] + 10;
	
    // 计算归一化偏差值
    float eleValue = (float)eleSub / (float)eleAdd * 100.0f;

	// 电感总和过低说明脱线（电感未检测到赛道），触发停车
	if(eleAdd < 20.0) Motor_SetStopFlag(1);
    else Motor_SetStopFlag(0);
    
    // 限幅处理：将偏差值限制在 -100 到 100 之间
    if(eleValue > 100.0f) eleValue = 100.0f;
    else if(eleValue < -100.0f) eleValue = -100.0f;

    // 转换为整型返回给控制层
    return (int)(eleValue);
}

/**
 * @brief 底盘闭环控制
 * 获取偏差并输入方向PID，计算差速输出
 */
void Chassis_Control(void)
{
    const PID_TypeDef *direction_pid; //外环：赛道偏差PID  
	
    // 2. 获取方向环PID控制器句柄和偏航角速度环PID句柄
    direction_pid = PID_GetController(PID_DIRECTION);
    
    // 3. 计算PID输出
    if (direction_pid != NULL) 
		{
				// 1. 获取当前的循迹偏差
				chassis.current_deviation = Calculate_Deviation();
        // 级联位置/方向PID计算，目标值设为0（即赛道中心）
        chassis.direction_output = PID_CascadePosition((PID_TypeDef *)direction_pid, chassis.current_deviation, 0);
    } else {
        // 如果PID未初始化，输出为0
        chassis.direction_output = 0;
    }
    // 左轮速度 = 基础速度 + 方向环修正
    chassis.left_speed = chassis.target_speed + chassis.direction_output;
    // 右轮速度 = 基础速度 - 转向修正
    chassis.right_speed = chassis.target_speed - chassis.direction_output;
    // 根据停车标志决定电机输出：停车时发送 0,0 停止电机
    if(Motor_GetData()->stop_flag) Motor_Control(0,0);
    else Motor_Control(chassis.left_speed, chassis.right_speed);
}

// 返回底盘数据的只读指针：外部通过 const Chassis_TypeDef* 访问速度、偏差、方向输出
const Chassis_TypeDef* Chassis_GetData(void)
{
    return &chassis;
}
