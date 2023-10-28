#include <math.h>
#include "DueCANLayer.h"
#include "shared/utils.h"


//************************** SUPER GENERIC FUNCTIONS ******************
//return the sign of a number
double sign(double num){
  if (num != 0){
    return num/abs(num);
  } else {
    return 1;
  }
}
