/**
 * @file SbusReceiver_sim.cpp
 * @brief Simulation implementation of SbusReceiver using HAL data layer
 *
 * This implementation provides the same interface as the real SbusReceiver
 * but gets its data from the HAL simulation layer instead of actual hardware.
 */

#include "SbusReceiver.h"
#include "hal/HALFactory.h"
#include <iostream>

SbusReceiver::SbusReceiver()
{
    // No hardware initialization needed for simulation
    std::cout << "SbusReceiver: Simulation mode initialized" << std::endl;
}

void SbusReceiver::init()
{
    // Ensure HAL RC receiver is available
    if (!HAL::rcReceiver) {
        std::cout << "SbusReceiver: Warning - HAL RC receiver not available" << std::endl;
    } else {
        std::cout << "SbusReceiver: Connected to HAL RC receiver" << std::endl;
    }
}

void SbusReceiver::read()
{
    // Update HAL RC receiver (this would normally be done by the HAL system)
    if (HAL::rcReceiver) {
        HAL::rcReceiver->update();
    }
}

int SbusReceiver::rcLost()
{
    if (!HAL::rcReceiver) {
        return 1; // Signal lost if HAL not available
    }
    return HAL::rcReceiver->isSignalLost() ? 1 : 0;
}

double SbusReceiver::getChannelData(SbusReceiverChannels channel, double defaultValue)
{
    if (!HAL::rcReceiver) {
        return defaultValue;
    }

    // Map SbusReceiverChannels to HAL RC receiver methods
    switch (channel) {
        case SbusReceiverChannels::LEFT_STICK_VERT:
            return HAL::rcReceiver->getLeftVertical();
        case SbusReceiverChannels::LEFT_STICK_HOR:
            return HAL::rcReceiver->getLeftHorizontal();
        case SbusReceiverChannels::RIGHT_STICK_VERT:
            return HAL::rcReceiver->getRightVertical();
        case SbusReceiverChannels::RIGHT_STICK_HOR:
            return HAL::rcReceiver->getRightHorizontal();
        case SbusReceiverChannels::LEFT_KNOB:
            return HAL::rcReceiver->getLeftKnob();
        case SbusReceiverChannels::RIGHT_KNOB:
            return HAL::rcReceiver->getRightKnob();
        case SbusReceiverChannels::RED_SWITCH:
            return HAL::rcReceiver->getRedSwitch();
        case SbusReceiverChannels::BLUE_SWITCH:
            return HAL::rcReceiver->getBlueSwitch();
        default:
            return defaultValue;
    }
}

double SbusReceiver::getBlueSwitch()
{
    return getChannelData(SbusReceiverChannels::BLUE_SWITCH);
}

bool SbusReceiver::isBlueSwitchUp()
{
    return getBlueSwitch() < -0.5;
}

bool SbusReceiver::isBlueSwitchDown()
{
    return getBlueSwitch() > 0.5;
}

bool SbusReceiver::isBlueSwitchCentered()
{
    double blueSwitch = getBlueSwitch();
    return blueSwitch > -0.5 && blueSwitch < 0.5;
}

double SbusReceiver::getHandheld()
{
    // Map to left knob for simulation
    return getChannelData(SbusReceiverChannels::LEFT_KNOB);
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
