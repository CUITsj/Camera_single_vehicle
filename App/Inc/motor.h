#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "common.h"
#include "include.h"

#define MOTOR_FTM FTM0
#define MOTOR1_PWM FTM_CH3
#define MOTOR2_PWM FTM_CH4
#define MOTOR3_PWM FTM_CH5
#define MOTOR4_PWM FTM_CH6
#define MOTOR_HZ (20*1000)
#define MOTOR_MIN 0//舵机最小值//右转1400
#define MOTOR_MAX 600 //舵机最大值//左转1700
#define MOTOR_STOPMAX 600

extern int32 MOTOR1_DUTY;
extern int32 MOTOR2_DUTY;
extern int16 MOTOR1_speed;
extern int16 MOTOR2_speed;

void speed_measure();
int32 MOTOR_protect(int32 duty, int32 min, int32 max);

#endif




