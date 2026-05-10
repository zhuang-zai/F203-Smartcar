#include "chassis.h"
#include "inductance.h"
#include "motor.h"
#include "LQ_Encoder.h"
#include "LQ_LSM6DSR_Hard.h"

Chassis_TypeDef chassis;
int16 direction_output;          // 内环输出：最终给车轮的差速修正量
int16 actual_yaw_rate;            // 内环输入：真实的偏航角速度
/**
 * @brief 底盘初始化
 * 初始化底盘结构体中的速度和目标偏差变量
 */
void Chassis_Init(void)
{
    chassis.target_speed = 500;       // 初始化目标速度为0
    chassis.current_deviation = 0;  // 初始化当前偏差为0
    chassis.left_speed = 0;         // 初始化左轮速度为0
    chassis.right_speed = 0;        // 初始化右轮速度为0
}

/**
 * @brief 获取陀螺仪Z轴角速度 (偏航角速度)
 * @return int16 Z轴原始角速度数据
 */
int16 Get_Yaw_Rate(void)
{
    int16 ax, ay, az, gx, gy, gz;
    
    // 调用底层函数读取6轴数据
    // 注意：传入的是变量的地址(&)，底层函数会把读到的值放进这些变量里
#ifdef HARDWARE_SPI
    LSM6DSR_Read_Data(&ax, &ay, &az, &gx, &gy, &gz);
#elif defined HARDWARE_IIC
    LQ_HARD_IIC_LSM60DSR_Read(&ax, &ay, &az, &gx, &gy, &gz);
#else
    // 如果都没有定义，默认给0防止报错，请检查你的工程宏定义
    gz = 0; 
#endif

    // 我们只需要 Z轴的角速度 (Gyro Z)
    return gz; 
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
    // 使用“差比和”可以消除由于速度变化导致的磁场强度整体变化影响
    int eleAdd = adc_values[0] + adc_values[1] + adc_values[2] + adc_values[3] + adc_values[4] + 10;
    
    // 计算归一化偏差值
    float eleValue = (float)eleSub / (float)eleAdd * 100.0f;
    
    // 限幅处理：将偏差值限制在 -100 到 100 之间
    if(eleValue > 100.0f) eleValue = 100.0f;
    else if(eleValue < -100.0f) eleValue = -100.0f;

    // 转换为整型返回给控制层
    return (int)(eleValue*10.0f);
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
        direction_output = PID_CascadePosition((PID_TypeDef *)direction_pid, chassis.current_deviation, 0);
    } else {
        // 如果PID未初始化，输出为0
        direction_output = 0;
    }
    //读取当前的偏航角速度
		actual_yaw_rate = Get_Yaw_Rate();
    // 4. 差速分配
    // 左轮速度 = 基础速度 + 转向修正
    chassis.left_speed = chassis.target_speed + direction_output;
    // 右轮速度 = 基础速度 - 转向修正
    chassis.right_speed = chassis.target_speed - direction_output;
    
    // 5. 执行电机控制
    Motor_Control(chassis.left_speed, chassis.right_speed);
}