#include "chassis.h"
#include "inductance.h"
#include "motor.h"
#include "LQ_Encoder.h"
#include "LQ_LSM6DSR_Hard.h"

// 设定单节 3.3V (总压 9.9V) 的极限死亡线阈值
#define BAT_SAFE_ADC_THRESHOLD  2240

Chassis_TypeDef chassis;
int16 direction_output;          // 内环输出：最终给车轮的差速修正量
int16 actual_yaw_rate;            // 内环输入：真实的偏航角速度
int16 current_adc;
int16 target_yaw_rate = 0;
// 定义在文件顶部或全局
float INNER_COEF = 1.2f; // 内轮减速系数 (建议范围: 1.0 ~ 1.5)
float OUTER_COEF = 0.3f; // 外轮增速系数 (建议范围: 0.1 ~ 0.4)

/**
 * @brief 底盘初始化
 * 初始化底盘结构体中的速度和目标偏差变量
 */
void Chassis_Init(void)
{
		chassis.target_speed = 0;       // 初始化目标速度为100   max : 1100
    chassis.current_deviation = 0;  // 初始化当前偏差为0
    chassis.left_speed = 0;         // 初始化左轮速度为0
    chassis.right_speed = 0;        // 初始化右轮速度为0
}

/**
 * @brief 电池低压保护监控任务
 * 建议放在 2ms 的 Timer1 中断中执行
 */
void Battery_Protection_Task(void)
{
    static uint16 low_vol_timer = 0;
    
    // 1. 直接读取原始 ADC 值 (无需消耗算力转换成浮点电压)
    current_adc = Get_ADCResult(ADC_CH9_P01);
    
		if(stop_flag == 1) return;
    // 2. 检测是否低于警戒线
    if (current_adc < BAT_SAFE_ADC_THRESHOLD) 
    {
        low_vol_timer++;
        
        // 3. 时间滤波（防抖）：2ms * 500 = 1秒
        // 防止电机瞬间加速抽血导致的误判
        if (low_vol_timer >= 500) 
        {
            stop_flag = 1;  // 触发全局急停
        }
    }
    else 
    {
        // 电压弹回安全线以上，清零计时器
        low_vol_timer = 0;
    }
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
	
		static uint16 off_track_timer = 0;   // 丢线持续时间计数器
		if(stop_flag == 1) return 0;
	
//		if(eleAdd < 20.0)
//		{
//			off_track_timer++;
//			if(off_track_timer >= 250) stop_flag = 1;
//		}
//		else
//		{
//			off_track_timer = 0;
//		}
    
    // 限幅处理：将偏差值限制在 -100 到 100 之间
    if(eleValue > 100.0f) eleValue = 100.0f;
    else if(eleValue < -100.0f) eleValue = -100.0f;

    // 转换为整型返回给控制层
    return (int)(eleValue);
}


/**
 * @brief 动态角速度阶跃测试序列 (状态机)
 * @note 依赖 Timer1 的 2ms 周期。500次 tick = 1秒。
 */
void Dynamic_Yaw_Test_Task(void)
{
    static uint16 time_tick = 0;   // 时间计数器
    static uint8 current_stage = 0; // 当前所处的测试阶段

    // 1. 如果车子处于急停/未发车状态，重置测试进度
    if (stop_flag == 1) 
    {
        time_tick = 0;
        current_stage = 0;
        target_yaw_rate = 0;
        return;
    }

    // 2. 车子启动后，开始计时 (2ms 进一次)
    time_tick++;

    // 3. 设定每个阶段持续的时间：例如 1.5 秒 (750 ticks) 切换一次状态
    if (time_tick >= 1500) 
    {
        time_tick = 0;       // 清零计时器
        current_stage++;     // 进入下一个测试阶段
    }

    // 4. 执行状态机序列
    switch (current_stage)
    {
        case 0: target_yaw_rate = 5000;  break;  // 阶段0：正转 3000
        case 1: target_yaw_rate = -5000; break;  // 阶段1：反转 3000
        case 2: target_yaw_rate = 6000;  break;  // 阶段2：正转 4000
        case 3: target_yaw_rate = -6000; break;  // 阶段3：反转 4000
        case 4: target_yaw_rate = 7000;  break;  // 阶段4：正转 5000
        case 5: target_yaw_rate = -7000; break;  // 阶段5：反转 5000
        case 6: target_yaw_rate = 8000;  break;  // 阶段6：正转 6000
        case 7: target_yaw_rate = -8000; break;  // 阶段7：反转 6000
			case 8: target_yaw_rate = 9000;break;
			case 9: target_yaw_rate = -9000;break;
			case 10:target_yaw_rate = 10000;break;
			case 11:target_yaw_rate = -10000;break;
        default:
            // 测试结束：归零并强行触发停车保护
            target_yaw_rate = 0;
            stop_flag = 1; 
            break;
    }
}

/**
 * @brief 底盘闭环控制
 * 获取偏差并输入方向PID，计算差速输出
 */
void Chassis_Control(void)
{
    const PID_TypeDef *direction_pid; //外环：赛道偏差PID  
		const PID_TypeDef *yaw_rate_pid;  // 内环：角速度PID (输出最终差速)
		static uint8 outer_loop_timer = 0;
		float left_coef;
    float right_coef;
    //获取方向环PID控制器句柄和偏航角速度环PID句柄
    direction_pid = PID_GetController(PID_DIRECTION);
		yaw_rate_pid = PID_GetController(PID_YAW_RATE); // 新增！获取第4组PID
		//电池保护
		Battery_Protection_Task();
		/*角速度环测试*/
//		Dynamic_Yaw_Test_Task();
  
//		outer_loop_timer++;
//    if(outer_loop_timer >= 3) 
//    {
//        outer_loop_timer = 0;
//        
//        chassis.current_deviation = Calculate_Deviation();
//        // 外环算出力矩，更新 direction_output
//        target_yaw_rate = PID_CascadePosition((PID_TypeDef *)direction_pid, chassis.current_deviation, 0);
//    }
//		direction_output = 0;
    //读取当前的偏航角速度
		actual_yaw_rate = Get_Yaw_Rate();
		
		direction_output = PID_CascadePosition((PID_TypeDef *)yaw_rate_pid, actual_yaw_rate, target_yaw_rate);
		
//		if (direction_output > 0) 
//    {
//        left_coef  = INNER_COEF; // 左转时，左轮是内侧
//        right_coef = OUTER_COEF; // 左转时，右轮是外侧
//    } 
//    else 
//    {
//        left_coef  = OUTER_COEF; // 右转时，左轮变外侧
//        right_coef = INNER_COEF; // 右转时，右轮变内侧
//    }

    // 如果是负数(右转)：左轮减去(负数*外侧) = 加速；右轮加上(负数*内侧) = 减速
//    chassis.left_speed  = chassis.target_speed - (int16)(direction_output * left_coef);
//    chassis.right_speed = chassis.target_speed + (int16)(direction_output * right_coef);
		
		chassis.left_speed  = chassis.target_speed - direction_output;
    chassis.right_speed = chassis.target_speed + direction_output;
    //执行电机控制
		Motor_Control(chassis.left_speed, chassis.right_speed);
}