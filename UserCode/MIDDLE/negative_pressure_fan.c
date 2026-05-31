#include "negative_pressure_fan.h"
#include "LQ_MotorServo.h"

static uint16 g_fan_duty = 0;

void Fan_Init(uint16 freq)
{
    BLmotor_Init_1(freq);
}

void Fan_SetSpeed(uint16 duty)
{
    /* 占空比下限钳位 */
    if (duty < FAN_MIN_DUTY) {
        duty = FAN_MIN_DUTY;
    }
    /* 占空比上限钳位 */
    if (duty > FAN_MAX_DUTY) {
        duty = FAN_MAX_DUTY;
    }
    g_fan_duty = duty;
    BLmotor_Ctrl_w1(duty);
}

uint16 Fan_GetSpeed(void)
{
    return g_fan_duty;
}

void Fan_Stop(void)
{
    g_fan_duty = 0;
    BLmotor_Ctrl_w1(0);
}