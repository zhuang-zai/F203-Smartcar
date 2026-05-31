
#ifndef __NEGATIVE_PRESSURE_FAN_H_
#define __NEGATIVE_PRESSURE_FAN_H_

#include "DEF.h"

#define FAN_MAX_DUTY        2000
#define FAN_MIN_DUTY        900

typedef enum {
    FAN_MODE_1 = 900,
    FAN_MODE_2 = 1100,
    FAN_MODE_3 = 1300,
    FAN_MODE_4 = 1500,
    FAN_MODE_5 = 2000
} FanSpeedMode;

void Fan_Init(uint16 freq);
void Fan_SetSpeed(uint16 duty);
void Fan_Stop(void);

uint16 Fan_GetSpeed(void);


#endif