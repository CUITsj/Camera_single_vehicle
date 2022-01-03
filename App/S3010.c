#include "S3010.h"

int32 S3010_DUTY = 1550;

uint16 S3010_protect(uint16 duty, uint16 min, uint16 max)
{
  if (duty >= max)
  {
    return max;
  }
  if (duty <= min)
  {
    return min;
  }
  else
  {
    return duty;
  }
}