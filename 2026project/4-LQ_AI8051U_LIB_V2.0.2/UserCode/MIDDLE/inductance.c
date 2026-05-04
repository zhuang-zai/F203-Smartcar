#include "inductance.h"

// 电磁传感器变量定义
static int ADC_values[5];//L1, L2, M, R1, R2

/**
 * @brief ADC 值归一化
 * 将 ADC 值归一化到指定范围
 * @param value ADC 值
 * @param min 最小值
 * @param max 最大值
 * @return 归一化后的值
 */
int _adcNormalize(int value, double min, double max)
{
    double normalized = (double)(value - min) / (max - min)* 100.0;
    if(normalized > 100.0) normalized = 100.0;
    if(normalized < 1.0) normalized = 1.0;
    return (int)normalized;
}

/**
 * @brief 扫描电磁传感器数据
 * 放在定时器读取五个电磁传感器ADC值
 */
void ScanInductance(void)
{
    // 原ADC值，调试用
    // ADC_values[0] = Get_ADCResult(5);//L1
    // ADC_values[1] = Get_ADCResult(1);//L2
    // ADC_values[2] = Get_ADCResult(2);//M
    // ADC_values[3] = Get_ADCResult(3);//R1
    // ADC_values[4] = Get_ADCResult(4);//R2

    //归一化后的ADC值
    ADC_values[0] = _adcNormalize(Get_ADCResult(5), 0, 2100);//L1
    ADC_values[1] = _adcNormalize(Get_ADCResult(1), 0, 2000);//L2
    ADC_values[2] = _adcNormalize(Get_ADCResult(2), 0, 2400);//M
    ADC_values[3] = _adcNormalize(Get_ADCResult(3), 0, 2100);//R1
    ADC_values[4] = _adcNormalize(Get_ADCResult(4), 0, 2200);//R2
}

int* GetInductance()
{
    return ADC_values;  // 返回数组首地址
}
