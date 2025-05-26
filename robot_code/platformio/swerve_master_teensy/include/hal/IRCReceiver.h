#ifndef _IRC_RECEIVER_H_
#define _IRC_RECEIVER_H_

#include "HALConfig.h"

/**
 * @brief RC Channel enumeration for standardized access
 */
enum class RCChannel {
    LEFT_VERTICAL = 0,    // Left stick vertical (forward/backward)
    LEFT_HORIZONTAL = 1,  // Left stick horizontal (left/right)
    RIGHT_VERTICAL = 2,   // Right stick vertical
    RIGHT_HORIZONTAL = 3, // Right stick horizontal
    LEFT_KNOB = 4,        // Left knob/dial
    RIGHT_KNOB = 5,       // Right knob/dial
    BLUE_SWITCH = 6,      // Mode switch (3-position)
    RED_SWITCH = 7,       // E-stop switch
    HANDHELD = 8,         // Handheld remote channel
    AUX1 = 9,            // Auxiliary channel 1
    AUX2 = 10,           // Auxiliary channel 2
    AUX3 = 11            // Auxiliary channel 3
};

/**
 * @brief RC receiver data structure
 */
struct RCData {
    static const int MAX_CHANNELS = 16;
    double channels[MAX_CHANNELS];
    bool signalLost;
    unsigned long lastUpdateTime;
    
    RCData() : signalLost(true), lastUpdateTime(0) {
        for (int i = 0; i < MAX_CHANNELS; i++) {
            channels[i] = 0.0;
        }
    }
};

/**
 * @brief Abstract interface for RC Receiver
 * 
 * This interface abstracts RC receiver operations to allow for both
 * real hardware (SBUS, PWM) and simulation implementations.
 */
class IRCReceiver {
public:
    virtual ~IRCReceiver() = default;
    
    /**
     * @brief Initialize the RC receiver
     * @return true if initialization successful
     */
    virtual bool begin() = 0;
    
    /**
     * @brief Update receiver data (call in main loop)
     */
    virtual void update() = 0;
    
    /**
     * @brief Get normalized channel data (-1.0 to 1.0)
     * @param channel The channel to read
     * @param defaultValue Default value if channel unavailable
     * @return Normalized channel value
     */
    virtual double getChannelData(RCChannel channel, double defaultValue = 0.0) = 0;
    
    /**
     * @brief Get raw channel data
     * @param channel The channel to read
     * @return Raw channel value
     */
    virtual int getRawChannelData(RCChannel channel) = 0;
    
    /**
     * @brief Check if signal is lost
     * @return true if signal is lost
     */
    virtual bool isSignalLost() = 0;
    
    /**
     * @brief Get time since last valid signal (microseconds)
     * @return Time since last signal
     */
    virtual unsigned long getTimeSinceLastSignal() = 0;
    
    /**
     * @brief Get all RC data at once
     * @return RCData structure with all channel data
     */
    virtual RCData getAllData() = 0;
    
    // Convenience methods for common controls
    virtual double getLeftVertical() = 0;
    virtual double getLeftHorizontal() = 0;
    virtual double getRightVertical() = 0;
    virtual double getRightHorizontal() = 0;
    virtual double getLeftKnob() = 0;
    virtual double getRightKnob() = 0;
    virtual int getBlueSwitch() = 0;  // -1, 0, 1 for 3-position switch
    virtual int getRedSwitch() = 0;   // 0 or 1 for 2-position switch
    virtual bool isBlueSwitchUp() = 0;
    virtual bool isBlueSwitchDown() = 0;
    virtual bool isBlueSwitchCentered() = 0;
};

#endif // _IRC_RECEIVER_H_
