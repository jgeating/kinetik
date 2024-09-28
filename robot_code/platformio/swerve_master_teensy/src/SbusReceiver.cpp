#include "SbusReceiver.h"
#include "sbus.h"

SbusReceiver::SbusReceiver() : m_sbusRx{bfs::SbusRx(&Serial2)}
{
}

void SbusReceiver::init()
{
    m_sbusRx.Begin();
}

void SbusReceiver::read()
{
    if (m_sbusRx.Read())
    {

        /* Grab the received data */
        m_data = m_sbusRx.data();
        
        if (!m_data.lost_frame) {
            lastDataReceiveTime = micros();
        }
    }
}

int SbusReceiver::rcLost() {
    uint32_t now = micros();
    bool timedOut = (now - lastDataReceiveTime) > RC_TIMEOUT;
    return timedOut ? 1 : 0;
}

double SbusReceiver::getChannelData(SbusReceiverChannels channel, double defaultValue = 0.0)
{
    int8_t channelNum = static_cast<int8_t>(channel);

    if (channelNum >= m_data.NUM_CH)
    {
        return defaultValue;
    }

    return (m_data.ch[channelNum] - CHANNEL_DATA_ZERO) / CHANNEL_DATA_MAGNITUDE;
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
