#include "motor.h"

int32 MOTOR1_DUTY = 0;//ÓÒ
int32 MOTOR2_DUTY = 0;//×ó
int16 MOTOR1_speed = 0;
int16 MOTOR2_speed = 0;

void speed_measure()
{
  int16 Pulses1, Pulses2;
  
  Pulses1 = ftm_quad_get(FTM1);
  ftm_quad_clean(FTM1);
  Pulses2 = ftm_quad_get(FTM2);
  ftm_quad_clean(FTM2);
  MOTOR1_speed = Pulses1;
  MOTOR2_speed = Pulses2;
  //var[1] = MOTOR1_speed;
  //var[2] = -MOTOR2_speed;
}
int32 MOTOR_protect(int32 duty, int32 min, int32 max)
{
  if (duty >= max)
  {
    return max;
  }
  else if (duty <= min)
  {
    return min;
  }
  else
  {
    return duty;
  }
}