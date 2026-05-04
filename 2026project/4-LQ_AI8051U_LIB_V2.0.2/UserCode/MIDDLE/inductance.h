#ifndef __INDUCTANCE_H_
#define __INDUCTANCE_H_

#include "AI8051U_ADC.h"
#include "string.h"
#include <stdio.h>
#include "include.h"
#include "LQ_ADC.h"

void ScanInductance(void);
int* GetInductance(void);

#endif