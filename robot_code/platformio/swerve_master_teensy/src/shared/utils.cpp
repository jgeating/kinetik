#include <math.h>
#include "shared/utils.h"
#include <cmath>
#include "Arduino.h"

//************************** SUPER GENERIC FUNCTIONS ******************
// return the sign of a number
double sign(double num)
{
  if (num != 0)
  {
    return num / abs(num);
  }
  else
  {
    return 1;
  }
}

double constr(double in, double low, double high)
{
  if (in < low)
  {
    return low;
  }
  if (in > high)
  {
    return high;
  }
  return in;
}

double dewrap(double x)
{ // unwrap angles to make within ±PI (±180 degrees). units are radians
  double ret = fmod(x + PI, TWO_PI);
  if (ret < 0)
    ret += TWO_PI;
  return  ret - PI;
}