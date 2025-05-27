/**
 * @file SbusReceiver.cpp
 * @brief Real hardware implementation of SbusReceiver using sbus.h
 *
 * This implementation provides the real hardware interface using the
 * Bolder Flight Systems SBUS library for actual SBUS communication.
 */

#include "SbusReceiver.h"
#include "sbus.h"

// Private implementation details for real hardware
class SbusReceiverImpl {
public:
  /* SBUS object, reading SBUS */
  bfs::SbusRx m_sbusRx;
  /* SBUS data */
  bfs::SbusData m_data;
  uint32_t lastDataReceiveTime = 0;
  uint32_t RC_TIMEOUT = 100000; // number of microseconds before receiver timeout is tripped

  const int16_t CHANNEL_DATA_MIN = 172;
  const int16_t CHANNEL_DATA_MAX = 1811;
  const int16_t CHANNEL_DATA_ZERO = 992;
  const double CHANNEL_DATA_MAGNITUDE = CHANNEL_DATA_ZERO - CHANNEL_DATA_MIN;

  SbusReceiverImpl() : m_sbusRx(&Serial2) {
    // Initialize SBUS receiver on Serial2
  }
};

static SbusReceiverImpl* impl = nullptr;

SbusReceiver::SbusReceiver() {
  if (!impl) {
    impl = new SbusReceiverImpl();
  }
}

void SbusReceiver::init() {
  impl->m_sbusRx.Begin();
}

void SbusReceiver::read() {
  if (impl->m_sbusRx.Read()) {
    /* Grab the received data */
    impl->m_data = impl->m_sbusRx.data();

    if (!impl->m_data.lost_frame) {
      impl->lastDataReceiveTime = micros();
    }
  }
}

int SbusReceiver::rcLost() {
  uint32_t now = micros();
  bool timedOut = (now - impl->lastDataReceiveTime) > impl->RC_TIMEOUT;
  return timedOut ? 1 : 0;
}

double SbusReceiver::getChannelData(SbusReceiverChannels channel, double defaultValue)
{
    if (rcLost()) {
        return defaultValue;
    }

    int8_t channelNum = static_cast<int8_t>(channel);

    if (channelNum >= impl->m_data.NUM_CH)
    {
        return defaultValue;
    }

    return (impl->m_data.ch[channelNum] - impl->CHANNEL_DATA_ZERO) / impl->CHANNEL_DATA_MAGNITUDE;
}

double SbusReceiver::getBlueSwitch()
{
    return getChannelData(SbusReceiverChannels::BLUE_SWITCH);
}

bool SbusReceiver::isBlueSwitchUp()
{
    return getBlueSwitch() < -.5;
}

bool SbusReceiver::isBlueSwitchDown()
{
    return getBlueSwitch() > .5;
}

bool SbusReceiver::isBlueSwitchCentered()
{
    double blueSwitch = getBlueSwitch();
    return blueSwitch > -.5 && blueSwitch < .5;
}

double SbusReceiver::getHandheld()
{
    return 0;
}

double SbusReceiver::getRedSwitch()
{
    return getChannelData(SbusReceiverChannels::RED_SWITCH);
}

double SbusReceiver::getRightKnob()
{
    return getChannelData(SbusReceiverChannels::RIGHT_KNOB);
}

double SbusReceiver::getLeftVert()
{
    return getChannelData(SbusReceiverChannels::LEFT_STICK_VERT);
}

double SbusReceiver::getLeftHor()
{
    return getChannelData(SbusReceiverChannels::LEFT_STICK_HOR);
}

double SbusReceiver::getRightVert()
{
    return getChannelData(SbusReceiverChannels::RIGHT_STICK_VERT);
}

double SbusReceiver::getRightHor()
{
    return getChannelData(SbusReceiverChannels::RIGHT_STICK_HOR);
}
