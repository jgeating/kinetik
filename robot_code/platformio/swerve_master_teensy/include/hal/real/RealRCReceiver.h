#ifndef _REAL_RC_RECEIVER_H_
#define _REAL_RC_RECEIVER_H_

#include "../HALConfig.h"

#if HAL_IMPLEMENTATION == HAL_REAL

#include "../IRCReceiver.h"
#include "SbusReceiver.h"

/**
 * @brief Real hardware implementation of RC receiver using SBUS
 */
class RealRCReceiver : public IRCReceiver {
private:
    SbusReceiver m_sbusReceiver;
    RCData m_data;
    bool m_initialized;
    
    // Channel mapping from SBUS to our standardized channels
    static const int CHANNEL_MAP[];

public:
    /**
     * @brief Constructor
     */
    RealRCReceiver();
    
    // IRCReceiver interface implementation
    bool begin() override;
    void update() override;
    double getChannelData(RCChannel channel, double defaultValue = 0.0) override;
    int getRawChannelData(RCChannel channel) override;
    bool isSignalLost() override;
    unsigned long getTimeSinceLastSignal() override;
    RCData getAllData() override;
    
    // Convenience methods
    double getLeftVertical() override;
    double getLeftHorizontal() override;
    double getRightVertical() override;
    double getRightHorizontal() override;
    double getLeftKnob() override;
    double getRightKnob() override;
    int getBlueSwitch() override;
    int getRedSwitch() override;
    bool isBlueSwitchUp() override;
    bool isBlueSwitchDown() override;
    bool isBlueSwitchCentered() override;

private:
    /**
     * @brief Convert SBUS channel to our standardized channel
     * @param sbusChannel SBUS channel number
     * @return Our standardized channel number, or -1 if invalid
     */
    int mapSbusChannel(int sbusChannel);
    
    /**
     * @brief Update internal data from SBUS receiver
     */
    void updateFromSbus();
    
    /**
     * @brief Convert 3-position switch value to standardized format
     * @param rawValue Raw switch value
     * @return -1, 0, or 1 for down, center, up
     */
    int convertThreePositionSwitch(double rawValue);
};

#endif // HAL_IMPLEMENTATION == HAL_REAL

#endif // _REAL_RC_RECEIVER_H_
