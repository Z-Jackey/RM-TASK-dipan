#ifndef __DJI_MOTOR_H
#define __DJI_MOTOR_H

#include "main.h"
#include "fdcan.h"

void DJI_Motor_SendCurrent(FDCAN_HandleTypeDef *hfdcan, int16_t i1, int16_t i2, int16_t i3, int16_t i4);

#endif