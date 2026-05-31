#ifndef __DEBUG_H_
#define __DEBUG_H_

#include "DEF.h"

// 调试模块初始化：初始化 UART 串口 + 从 EEPROM 加载 PID 参数
void Debug_Init(void);
// 通过 UART 发送调试数据帧（FireWater 协议）：电感、速度、编码器、PWM、偏差、陀螺仪
void Debug_SendFrame(void);
// 通过 UART 发送当前所有 PID 参数（15 个 float，发送字符 'R' 触发）
void Debug_SendPIDParams(void);
// 处理 UART 接收字节：检测 'R' 命令 + VOFA+ JustFloat 调参帧解析
void Debug_ProcessRxByte(uint8 byte);
// 将当前 PID 参数保存到 EEPROM（调参后自动调用，受 eeprom_enabled 控制）
uint8 Debug_SaveToEEPROM(void);
// 从 EEPROM 加载 PID 参数（初始化时调用，受 eeprom_enabled 控制）
uint8 Debug_LoadFromEEPROM(void);


#endif
