#ifndef __PID_H__
#define __PID_H__

#include "common.h"
#include "include.h"

#define KP 0
#define KI 1
#define KD 2

typedef struct PID
{
        long sum_error;
	int32 LastError;	//Error[-1]
} PID;

extern PID S3010_PID, MOTOR_PID;
extern  int16 var[6];

//舵机PID初始化
void S3010PID_Init(PID *sptr);

//电机PID初始化
void MOTORPID_Init(PID *sptr);

//舵机PID调节
int32 S3010PID_Control(PID *sptr, int32 TrackPoint, int32 ScreenPoint);

//电机PID调节
int32 MOTORPID_Control(PID *sptr, int32 SetSpeed, int32 NowSpeed);

#endif