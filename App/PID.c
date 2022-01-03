#include "PID.h"

PID S3010_PID, MOTOR_PID;
float PID_S3010[3][3] = {{0, 0, 14}, {0, 0, 1.3}, {0, 0, 1.5}};
float PID_MOTOR[3] = {12,1.9,5};//0.9\1\0.8
int8 set = 0; //舵机PID参数设置

int16 var[6];

void S3010PID_Init(PID *sptr)
{
  sptr->LastError = 0;
}
void MOTORPID_Init(PID *sptr)
{
  sptr->sum_error = 0;
  sptr->LastError = 0;
}
int32 S3010PID_Control(PID *sptr, int32 TrackPoint, int32 ScreenPoint)
{
  register int32 iError, OutActual;//register 
  float Kp;
  iError =  ScreenPoint - TrackPoint;
  Kp = (float)iError*iError/60 + 9;
  OutActual = (int32)(S3010_MID + Kp * iError + PID_S3010[set][KD] *(iError - sptr->LastError));
  sptr->LastError = iError;
  return OutActual;
  
}
int32 MOTORPID_Control(PID *sptr, int32 SetSpeed, int32 NowSpeed)
{
  register int32 iError, Increase;
  
  iError = SetSpeed - NowSpeed;  
  sptr->sum_error += iError;
 // sptr->sum_error = (iError + sptr->LastError)/2;
	if (sptr->sum_error >= 1000)
	{
		sptr->sum_error = 1000;
	}
	else if (sptr->sum_error <= -1000)
	{
		sptr->sum_error = -1000;
	}
  Increase = (int32)(PID_MOTOR[KP]*iError + PID_MOTOR[KI]*sptr->sum_error + PID_MOTOR[KD]*(iError - sptr->LastError));
  sptr->LastError = iError;
  if (Increase > 400)
  {
    Increase = 600;
  }
  if (Increase < -400)
  {
    Increase = -600;
  }
  return Increase;
  
}

