#ifndef __S3010_H__
#define __S3010_H__

#include "common.h"
#include "include.h"

#define S3010_FTM FTM3
#define S3010_CH FTM_CH0
#define S3010_HZ 100
#define S3010_MID 1550 //舵机中值//中间
#define S3010_MIN 1400 //舵机最小值//右转1370
#define S3010_MAX 1700 //舵机最大值//左转1730

extern int32 S3010_DUTY;

uint16 S3010_protect(uint16 duty, uint16 min, uint16 max);

#endif