#include "debug.h"
#include "chassis.h"
#include "motor.h"
#include "inductance.h"
#include "IMU.h"
#include "LQ_UART.h"
#include "AI8051U_EEPROM.h"
#include "pid.h"

// debug.c 模块新增代码注释

// 串口波特率定义：115200 用于蓝牙与电脑 VOFA+ 通信
#define DEBUG_UART_BAUD   115200
// UART1 引脚选择：P30(TX) P31(RX)
#define DEBUG_UART_PIN    UART1_P30_P31

// EEPROM 存储 PID 参数的起始地址（扇区 0）
#define EEPROM_PID_BASE   0x0000
// EEPROM 魔数标记：用于校验 EEPROM 数据是否有效
#define EEPROM_MAGIC      0x5AA5

// 调试功能使能标志：1=开启 0=关闭，定时器中断发送前会检查此标志
static uint8 debug_enabled = 0;
// EEPROM 功能使能标志：1=开启保存/加载 0=关闭所有 EEPROM 操作
static uint8 eeprom_enabled = 0;
// UART 发送缓冲区：80 字节，满足 Debug_SendFrame 的 33 字节和 Debug_SendPIDParams 的 65 字节
static uint8 tx_buf[80];
// UART 接收缓冲区：32 字节，用于缓存接收到的VOFA+调参帧
static uint8 rx_buf[32];
// 接收计数器：记录rx_buf中已缓存的字节数
static uint16 rx_cnt = 0;

// EEPROM 存储结构体：包含魔数+15个PID参数浮点值（3个PID×5个参数）
typedef struct
{
    uint16 magic;
    float params[15];
} EEPROM_PID_Data;

void Debug_Init(void)
{
    // 初始化 UART1 串口，连接蓝牙模块用于调试通信
    UART_Init(DEBUG_UART_PIN, DEBUG_UART_BAUD);
    debug_enabled = 1;
    rx_cnt = 0;

    // 尝试从 EEPROM 加载 PID 参数：失败则使用默认值
    if(Debug_LoadFromEEPROM() == 0)
    {
        PID_LoadDefaults();
    }
}

void Debug_SendFrame(void)
{
    u16 idx;        // 缓冲区写入索引
    u8 chk;         // 异或校验值
    int *ind;       // 归一化电感值数组指针
    const MotorData *mot;       // 电机数据只读指针（编码器、PWM等）
    const Chassis_TypeDef *chs; // 底盘数据只读指针（目标速度、偏差等）
    const IMU_Data_TypeDef *imu;// 陀螺仪数据只读指针

    // 调试未启用则直接返回，避免不必要的串口通信
    if (debug_enabled == 0) return;

    // 从各模块获取最新数据
    ind = GetInductance();
    mot = Motor_GetData();
    chs = Chassis_GetData();
    imu = IMU_GetData();

    /* VOFA+ FireWater协议: 0xAA 0x55 0x5A 0xA5 + 14*2bytes int16 + 异或校验 */
    tx_buf[0] = 0xAA;
    tx_buf[1] = 0x55;
    tx_buf[2] = 0x5A;
    tx_buf[3] = 0xA5;
    idx = 4;

    /* 每个int16转为2字节发送: 低字节在前 */
    /* 0-4: 归一化电感值 L1 L2 M R1 R2 */
    tx_buf[idx++] = (u8)(ind[0]);
    tx_buf[idx++] = (u8)(ind[0] >> 8);
    tx_buf[idx++] = (u8)(ind[1]);
    tx_buf[idx++] = (u8)(ind[1] >> 8);
    tx_buf[idx++] = (u8)(ind[2]);
    tx_buf[idx++] = (u8)(ind[2] >> 8);
    tx_buf[idx++] = (u8)(ind[3]);
    tx_buf[idx++] = (u8)(ind[3] >> 8);
    tx_buf[idx++] = (u8)(ind[4]);
    tx_buf[idx++] = (u8)(ind[4] >> 8);

    /* 5: 左轮目标速度 */
    tx_buf[idx++] = (u8)(chs->left_speed);
    tx_buf[idx++] = (u8)(chs->left_speed >> 8);
    /* 6: 左轮编码器实际值 */
    tx_buf[idx++] = (u8)(mot->encoder_L);
    tx_buf[idx++] = (u8)(mot->encoder_L >> 8);
    /* 7: 左轮PWM输出 */
    tx_buf[idx++] = (u8)(mot->left_pwm);
    tx_buf[idx++] = (u8)(mot->left_pwm >> 8);

    /* 8: 右轮目标速度 */
    tx_buf[idx++] = (u8)(chs->right_speed);
    tx_buf[idx++] = (u8)(chs->right_speed >> 8);
    /* 9: 右轮编码器实际值 */
    tx_buf[idx++] = (u8)(mot->encoder_R);
    tx_buf[idx++] = (u8)(mot->encoder_R >> 8);
    /* 10: 右轮PWM输出 */
    tx_buf[idx++] = (u8)(mot->right_pwm);
    tx_buf[idx++] = (u8)(mot->right_pwm >> 8);

    /* 11: 赛道偏差(方向环输入) */
    tx_buf[idx++] = (u8)(chs->current_deviation);
    tx_buf[idx++] = (u8)(chs->current_deviation >> 8);
    /* 12: 方向环PID输出 */
    tx_buf[idx++] = (u8)(chs->direction_output);
    tx_buf[idx++] = (u8)(chs->direction_output >> 8);
    /* 13: 陀螺仪偏航角速度 */
    tx_buf[idx++] = (u8)(imu->gyro_z);
    tx_buf[idx++] = (u8)(imu->gyro_z >> 8);

    /* 计算异或校验(从header到数据结束) */
    chk = 0;
    for (idx = 0; idx < 4 + 14 * 2; idx++)
    {
        chk ^= tx_buf[idx];
    }
    tx_buf[idx] = chk;

    /* 发送整帧数据 */
    for (idx = 0; idx < 4 + 14 * 2 + 1; idx++)
    {
        UART1_PutBuf(tx_buf[idx]);
    }
}

// 向 VOFA+ 发送当前所有 PID 参数
// 用于上位机读取当前调参状态，发送字符 'R' 时触发
// 帧格式: 0xAA 0x55 0x5A 0xA5(4字节帧头) + 15个float(60字节) + 异或校验(1字节) = 65字节
// 15个float顺序: 左环(Kp,Ki,Kd,out_max,out_min) + 右环(Kp,Ki,Kd,out_max,out_min) + 方向环(Kp,Ki,Kd,out_max,out_min)
void Debug_SendPIDParams(void)
{
    u8 idx;         // 发送缓冲区写入索引
    u8 chk;         // 异或校验累加值
    u8 pid_idx;     // PID 控制器索引：0=左环, 1=右环, 2=方向环
    u8 param_type;  // 参数类型索引：0=Kp, 1=Ki, 2=Kd, 3=out_max, 4=out_min
    float param_val;// 当前读取的 PID 参数值（float）
    u8 *fp;         // float 字节指针，用于逐字节拆分 float 为小端序

    // 写入 4 字节 JustFloat 帧头，VOFA+ 识别用
    tx_buf[0] = 0xAA;
    tx_buf[1] = 0x55;
    tx_buf[2] = 0x5A;
    tx_buf[3] = 0xA5;
    idx = 4;

    // 遍历 3 个 PID 控制器，每个控制器 5 个参数
    for(pid_idx = 0; pid_idx < 3; pid_idx++)
    {
        for(param_type = 0; param_type < 5; param_type++)
        {
            // 通过 PID_GetParam 获取当前参数值
            param_val = PID_GetParam((PID_Index)pid_idx, param_type);
            // 将 float 地址强转为字节指针，逐字节按小端序写入缓冲区
            fp = (u8 *)&param_val;
            tx_buf[idx++] = fp[0];
            tx_buf[idx++] = fp[1];
            tx_buf[idx++] = fp[2];
            tx_buf[idx++] = fp[3];
        }
    }

    // 计算异或校验：从帧头到最后一个数据字节逐字节异或
    chk = 0;
    for(idx = 0; idx < 4 + 15 * 4; idx++)
    {
        chk ^= tx_buf[idx];
    }
    tx_buf[idx] = chk;

    // 通过 UART1 发送整帧数据（帧头+数据+校验）
    for(idx = 0; idx < 4 + 15 * 4 + 1; idx++)
    {
        UART1_PutBuf(tx_buf[idx]);
    }
}

// 处理 UART 接收到的单个字节，用于 VOFA+ 调参帧解析和 'R' 命令触发
// 在 UART1 中断中被调用，每次接收一个字节就调用一次
void Debug_ProcessRxByte(uint8 byte)
{
    u8 i;         // 循环索引，用于校验计算
    float val;    // 解析出的 PID 索引值
    uint8 *fp;    // float 字节指针

    // 检测字符 'R'（Read）：收到 'R' 时立即回传所有 PID 参数
    if(byte == 'R')
    {
        Debug_SendPIDParams();
        return;
    }

    // 将接收到的字节存入缓冲区，防止溢出（最大 32 字节）
    if(rx_cnt < 32)
    {
        rx_buf[rx_cnt++] = byte;
    }

    /* 检测 JustFloat 帧头 0xAA 0x55 0x5A 0xA5 */
    /* 原理：滑动窗口检测，检查缓冲区末尾 4 个字节是否匹配帧头 */
    if(rx_cnt >= 4)
    {
        if(rx_buf[rx_cnt-4] == 0xAA && rx_buf[rx_cnt-3] == 0x55 &&
           rx_buf[rx_cnt-2] == 0x5A && rx_buf[rx_cnt-1] == 0xA5)
        {
            // 检测到帧头，重置计数器从帧头后开始计数
            rx_cnt = 0;
        }
    }

    /* 完整帧: 4字节帧头 + 16字节数据(4个float) + 1字节校验 = 21字节 */
    if(rx_cnt >= 21)
    {
        /* 校验：逐字节异或前 20 个字节 */
        uint8 chk = 0;
        for(i = 0; i < 20; i++)
        {
            chk ^= rx_buf[i];
        }

        // 校验通过，开始解析数据
        if(chk == rx_buf[20])
        {
            /* 解析 4 个 float (小端) */
            /* fp[0..3] = PID索引(float转int), fp[4..7] = 参数类型(0=Kp..4=out_min), fp[8..11] = 参数值 */
            fp = &rx_buf[4];
            val = *(float *)fp;
            PID_SetParam((PID_Index)(val + 0.5f), (uint8)(*(float *)(fp + 4) + 0.5f), *(float *)(fp + 8));

            // 调参后立即保存到 EEPROM，掉电不丢失
            Debug_SaveToEEPROM();
        }

        // 处理完毕，重置计数器等待下一帧
        rx_cnt = 0;
    }
}

// 将当前 PID 参数保存到 EEPROM，实现掉电不丢失
// 在每次 VOFA+ 调参后调用，覆盖 EEPROM 扇区 0
// 受 eeprom_enabled 标志位控制：为 0 时直接返回不执行任何操作
uint8 Debug_SaveToEEPROM(void)
{
    EEPROM_PID_Data eep;
    uint16 idx;

    // EEPROM 功能关闭时直接返回，不执行擦除和写入操作
    if(eeprom_enabled == 0) return 0;

    // 写入魔数标记，用于加载时校验数据有效性
    eep.magic = EEPROM_MAGIC;

    // 遍历 3 个 PID 控制器，每个控制器 5 个参数，按顺序存入 params 数组
    for(idx = 0; idx < 3; idx++)
    {
        eep.params[idx * 5 + 0] = PID_GetParam((PID_Index)idx, 0);  // Kp
        eep.params[idx * 5 + 1] = PID_GetParam((PID_Index)idx, 1);  // Ki
        eep.params[idx * 5 + 2] = PID_GetParam((PID_Index)idx, 2);  // Kd
        eep.params[idx * 5 + 3] = PID_GetParam((PID_Index)idx, 3);  // output_max
        eep.params[idx * 5 + 4] = PID_GetParam((PID_Index)idx, 4);  // output_min
    }

    // 先擦除 EEPROM 扇区，再写入数据
    EEPROM_SectorErase(EEPROM_PID_BASE);
    EEPROM_write_n(EEPROM_PID_BASE, (uint8 *)&eep, sizeof(eep));

    return 1;
}

// 从 EEPROM 加载 PID 参数到运行时内存
// 在 Debug_Init 中调用，如果 EEPROM 无有效数据则返回 0
// 受 eeprom_enabled 标志位控制：为 0 时直接返回 0（不调用则 Debug_Init 会走默认值分支）
uint8 Debug_LoadFromEEPROM(void)
{
    EEPROM_PID_Data eep;

    // EEPROM 功能关闭时直接返回 0，调用方会走默认值分支
    if(eeprom_enabled == 0) return 0;

    // 从 EEPROM 读取数据到临时结构体
    EEPROM_read_n(EEPROM_PID_BASE, (uint8 *)&eep, sizeof(eep));

    // 校验魔数：不匹配说明 EEPROM 未写入过有效 PID 数据
    if(eep.magic != EEPROM_MAGIC)
    {
        return 0;
    }

    // 将 EEPROM 中的参数逐个写入运行时 PID 控制器
    // 左轮速度环 (PID_LEFT, params[0..4])
    PID_SetParam(PID_LEFT, 0, eep.params[0]);
    PID_SetParam(PID_LEFT, 1, eep.params[1]);
    PID_SetParam(PID_LEFT, 2, eep.params[2]);
    PID_SetParam(PID_LEFT, 3, eep.params[3]);
    PID_SetParam(PID_LEFT, 4, eep.params[4]);

    // 右轮速度环 (PID_RIGHT, params[5..9])
    PID_SetParam(PID_RIGHT, 0, eep.params[5]);
    PID_SetParam(PID_RIGHT, 1, eep.params[6]);
    PID_SetParam(PID_RIGHT, 2, eep.params[7]);
    PID_SetParam(PID_RIGHT, 3, eep.params[8]);
    PID_SetParam(PID_RIGHT, 4, eep.params[9]);

    // 方向环 (PID_DIRECTION, params[10..14])
    PID_SetParam(PID_DIRECTION, 0, eep.params[10]);
    PID_SetParam(PID_DIRECTION, 1, eep.params[11]);
    PID_SetParam(PID_DIRECTION, 2, eep.params[12]);
    PID_SetParam(PID_DIRECTION, 3, eep.params[13]);
    PID_SetParam(PID_DIRECTION, 4, eep.params[14]);

    return 1;
}
