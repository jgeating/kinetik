#ifndef _SBUSRECEIVER_
#define _SBUSRECEIVER_

#include "sbus.h"

enum class SbusReceiverChannels : int8_t
{
  LEFT_STICK_VERT = 0,
  LEFT_STICK_HOR = 3,
  RIGHT_STICK_VERT = 2,
  RIGHT_STICK_HOR = 1,
  LEFT_KNOB = 4,
  RIGHT_KNOB = 5,
  RED_SWITCH = 6,
  BLUE_SWITCH = 7,
};

class SbusReceiver
{
private:
  /* SBUS object, reading SBUS */
  bfs::SbusRx m_sbusRx;
  /* SBUS data */
  bfs::SbusData m_data;

  const int16_t CHANNEL_DATA_MIN = 172;
  const int16_t CHANNEL_DATA_MAX = 1811;
  const int16_t CHANNEL_DATA_ZERO = 992;

  double getChannelData(SbusReceiverChannels channel, int16_t defaultValue = -1);

public:
  SbusReceiver();
  void read();

  double getBlueSwitch();
  bool isBlueSwitchUp();
  bool isBlueSwitchDown();
  bool isBlueSwitchCentered();
  double getHandheld();
  double getRedSwitch();
  double getRightKnob();
  double getLeftVert();
  double getLeftHor();
  double getRightVert();
  double getRightHor();
};

/********************************************************************/
#endif // _SBUSRECEIVER_