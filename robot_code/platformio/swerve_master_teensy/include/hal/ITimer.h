#ifndef _ITIMER_H_
#define _ITIMER_H_

#include <stdint.h>

/**
 * @brief Interface for timing and delay operations
 * 
 * This interface provides platform-independent access to timing functions.
 * It abstracts Arduino functions like micros(), millis(), delay(), and delayMicroseconds().
 */
class ITimer {
public:
    virtual ~ITimer() = default;
    
    /**
     * @brief Get microseconds since program start
     * @return Microseconds elapsed
     */
    virtual uint32_t micros() = 0;
    
    /**
     * @brief Get milliseconds since program start
     * @return Milliseconds elapsed
     */
    virtual uint32_t millis() = 0;
    
    /**
     * @brief Delay execution for specified milliseconds
     * @param ms Milliseconds to delay
     */
    virtual void delay(uint32_t ms) = 0;
    
    /**
     * @brief Delay execution for specified microseconds
     * @param us Microseconds to delay
     */
    virtual void delayMicroseconds(uint32_t us) = 0;
    
    /**
     * @brief Initialize the timer system
     * @return true if initialization successful
     */
    virtual bool begin() = 0;
    
    /**
     * @brief Check if timer system is ready
     * @return true if ready for operations
     */
    virtual bool isReady() const = 0;
    
    /**
     * @brief Get high-resolution timestamp
     * @return High-resolution timestamp in microseconds
     */
    virtual uint64_t getHighResTime() = 0;
    
    /**
     * @brief Reset the timer reference point
     * Useful for relative timing measurements
     */
    virtual void reset() = 0;
};

#endif // _ITIMER_H_
