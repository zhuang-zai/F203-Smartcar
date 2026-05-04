#include "pid.h"

#define PID_GROUPS 3 // 可根据需要调整组数
static PID_TypeDef pid_array[PID_GROUPS];

// PID参数数组，按顺序：Kp, Ki, Kd, out_max, out_min,mode
static const float pid_param_table[PID_GROUPS][6] = 
{
    //左轮速度环 增量式PI
    {9.0f, 3.5f, 0.0f, 3000,-3000,PID_INCREMENT},
    //右轮速度环 增量式PI
    {9.0f, 3.5f, 0.0f, 3000,-3000,PID_INCREMENT},
    //方向环 位置式PD
    {0.65, 0.0f, 1.88f, 1200,-1200, PID_POSITION}
};

// 初始化所有PID结构体
void PID_AllInit(void)
{
    int i;
    for(i=0; i<PID_GROUPS; i++)
    {
        PID_Init(&pid_array[i],
            pid_param_table[i][0],
            pid_param_table[i][1],
            pid_param_table[i][2],
            pid_param_table[i][3],
            pid_param_table[i][4],
            (PID_Mode)((int)pid_param_table[i][5])
        );
    }
}

// PID初始化，支持位置式和增量式
void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd, float out_max, float out_min,PID_Mode mode)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->prev_error = 0;
    pid->prev_prev_error = 0;
    pid->integral = 0;
    pid->output_max = out_max;
    pid->output_min = out_min;
    pid->output = 0;
    pid->mode = mode;
}

// PID计算函数（通过索引调用）
int16 PID_Calculate_ByIndex(PID_Index index, int16 current, int16 target)
{
    if (index >= 0 && index < PID_GROUPS) {
        PID_TypeDef *pid = &pid_array[index];
        if (pid->mode == PID_POSITION) {
            return PID_CascadePosition(pid, current, target);
        } else {
            return PID_CascadeIncrement(pid, current, target);
        }
    }
    return 0;
}

// 串级控制 - 位置式PID
int16 PID_CascadePosition(PID_TypeDef *pid, int16 current, int16 target)
{
    float error, derivative, output;
    error = (float)target - (float)current;
    
    // 积分项
    pid->integral += error;
    
    // 微分项
    derivative = error - pid->prev_error;
    
    // PID计算
    output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    
    // 输出限幅
    if(output > pid->output_max) output = pid->output_max;
    if(output < pid->output_min) output = pid->output_min;
    
    // 更新历史误差
    pid->prev_prev_error = pid->prev_error;
    pid->prev_error = error;
    
    return (int16)output;
}

// 串级控制 - 增量式PID
int16 PID_CascadeIncrement(PID_TypeDef *pid, int16 current, int16 target)
{
    float error, delta, output;
    error = (float)target - (float)current;
    
    // 增量计算
    delta = pid->Kp * (error - pid->prev_error)
                + pid->Ki * error
                + pid->Kd * (error - 2 * pid->prev_error + pid->prev_prev_error);
    
    // 更新输出
    output = pid->output + delta;
    pid->output = output;
    
    // 输出限幅
    if(output > pid->output_max) output = pid->output_max;
    if(output < pid->output_min) output = pid->output_min;
    
    // 更新历史误差
    pid->prev_prev_error = pid->prev_error;
    pid->prev_error = error;
    
    return (int16)output;
}

// 获取特定PID控制器的只读指针
const PID_TypeDef* PID_GetController(PID_Index index)
{
    if (index >= 0 && index < PID_GROUPS) {
        return &pid_array[index];
    }
    return NULL;
}