#ifndef _IADC_H_
#define _IADC_H_

#include <stdint.h>

/**
 * @brief Interface for Analog-to-Digital Converter operations
 * 
 * This interface provides platform-independent access to analog input pins.
 * It abstracts Arduino functions like analogRead() and analogReadResolution().
 */
class IADC {
public:
    virtual ~IADC() = default;
    
    /**
     * @brief Read an analog value from a pin
     * @param pin Analog pin number
     * @return Analog value (0 to max resolution value)
     */
    virtual uint16_t analogRead(uint8_t pin) = 0;
    
    /**
     * @brief Set the resolution of analog readings
     * @param bits Number of bits for resolution (8, 10, 12, 16)
     */
    virtual void setResolution(uint8_t bits) = 0;
    
    /**
     * @brief Get the current resolution setting
     * @return Number of bits for current resolution
     */
    virtual uint8_t getResolution() const = 0;
    
    /**
     * @brief Get the maximum value for current resolution
     * @return Maximum analog value (e.g., 4095 for 12-bit)
     */
    virtual uint16_t getMaxValue() const = 0;
    
    /**
     * @brief Initialize the ADC system
     * @return true if initialization successful
     */
    virtual bool begin() = 0;
    
    /**
     * @brief Check if ADC system is ready
     * @return true if ready for operations
     */
    virtual bool isReady() const = 0;
    
    /**
     * @brief Convert analog reading to voltage
     * @param reading Raw analog reading
     * @param referenceVoltage Reference voltage (default 3.3V)
     * @return Voltage value
     */
    virtual float toVoltage(uint16_t reading, float referenceVoltage = 3.3f) const = 0;
};

#endif // _IADC_H_
