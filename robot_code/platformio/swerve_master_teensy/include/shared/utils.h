#ifndef _SWERVE_UTILS_
#define _SWERVE_UTILS_

//************************** SUPER GENERIC FUNCTIONS ******************
//return the sign of a number
double sign(double num);

//constrain a number within a range
double constr(double in, double low, double high);

double dewrap(double x);

typedef enum {
  CAN_PACKET_SET_DUTY = 0,
  CAN_PACKET_SET_CURRENT,
  CAN_PACKET_SET_CURRENT_BRAKE,
  CAN_PACKET_SET_RPM,
  CAN_PACKET_SET_POS,
  CAN_PACKET_FILL_RX_BUFFER,
  CAN_PACKET_FILL_RX_BUFFER_LONG,
  CAN_PACKET_PROCESS_RX_BUFFER,
  CAN_PACKET_PROCESS_SHORT_BUFFER,
  CAN_PACKET_STATUS
} CAN_PACKET_ID;

#endif
