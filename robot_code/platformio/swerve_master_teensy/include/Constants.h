#ifndef _CONSTANTS_
#define _CONSTANTS_

#include "hal/HALConfig.h"

#if HAL_IMPLEMENTATION == HAL_REAL
#include <FlexCAN_T4.h>
#define CANBUS CAN3
#elif HAL_IMPLEMENTATION == HAL_SIM
// Mock CAN bus for simulation
#define CANBUS 0
#endif

/********************************************************************/
#endif // _CONSTANTS_