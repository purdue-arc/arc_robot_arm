#ifndef UTILS_H
#define UTILS_H

inline float RadiansToDegrees(float position_radians)
{
  return position_radians * 57.2958;
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#endif
