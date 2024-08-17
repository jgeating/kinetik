#ifndef _PWMRECEIVER_
#define _PWMRECEIVER_

#include "Arduino.h"
#include "shared/Channel.h" // for RC PWM inputs

const int CHANNEL_PIN[] = {
    38, // left stick vertical, forward = (+)
    40, // right stick horizontal, R = (+)
    42, // right stick vertical, forward = (+)
    44, // left stick horizontal, R = (+)
    46, // left knob, CW = (+)
    48, // right knob, CW = (+)
    50, // left switch, backwards = (-)
    52  // right switch, backwards = (-)
};

class PWMReceiver
{
private:
public:
  short chs = 8;                                                     // number of channels to read from receiver
  short chOff[8] = {1500, 1500, 1500, 1500, 1471, 1500, 1500, 1500}; // Channel offsets (calibrate to find these)
  Channel **channels = new Channel *[chs];
  u_int32_t rcTimeout = 100000; // number of microseconds before receiver timeout is tripped - make sure it is a good bit longer than 2000 microseconds
  bool rcLost = 1;              // This is set to true if no receiver signal is received within the rcTimeout timeframe (microseconds)
  // red switch
  int estop_ch = 6; // which Rx channel is used for motor enabling/SW e-stop. 0 index
  // blue switch
  int mode_ch = 7;   // which Rx channel is used for setting mode
  int remote_ch = 4; // Chick Rx channel is hooked up to the e-skate remote (for imu/pad control)
  int remote_max_3 = 355;
  int remote_max_2 = 217;
  int remote_max_1 = 164;

  PWMReceiver()
  {
    for (int i = 0; i < chs; i++)
    {
      channels[i] = new Channel(CHANNEL_PIN[i], chOff[i]);
    }
  }

  int getBlueSwitch()
  {
    return channels[mode_ch]->getCh();
  }

  bool isBlueSwitchUp()
  {
    return channels[mode_ch]->getCh() < -150;
  }

  bool isBlueSwitchDown()
  {
    return channels[mode_ch]->getCh() > 150;
  }

  bool isBlueSwitchCentered()
  {
    return channels[mode_ch]->getCh() > -150 && channels[mode_ch]->getCh() < 150;
  }

  int getHandheld()
  {
    return channels[remote_ch]->getCh();
  }

  int getRedSwitch()
  {
    return channels[estop_ch]->getCh();
  }

  // int getLeftKnob()
  // {
  //   return channels[4]->getCh();
  // }

  int getRightKnob()
  {
    return channels[5]->getCh();
  }

  int getLeftVert()
  {
    return channels[0]->getCh();
  }

  int getLeftHor()
  {
    return channels[3]->getCh();
  }

  int getRightVert()
  {
    return channels[2]->getCh();
  }

  int getRightHor()
  {
    return channels[1]->getCh();
  }
};

#endif