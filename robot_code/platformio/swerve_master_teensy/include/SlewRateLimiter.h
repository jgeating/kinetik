#ifndef _SLEW_RATE_LIMITER_
#define _SLEW_RATE_LIMITER_

#include <Arduino.h>

/**
 * A class that limits the rate of change of an input value. Useful for implementing voltage,
 * setpoint, and/or output ramps. A slew-rate limit is most appropriate when the quantity being
 * controlled is a velocity or a voltage; when controlling a position, consider using a
 * trapezoidal profile instead.
 */
class SlewRateLimiter {
private:
    double m_positiveRateLimit;
    double m_negativeRateLimit;
    double m_prevVal;
    unsigned long m_prevTime;

    /**
     * Clamp a value between min and max bounds
     */
    double clamp(double value, double min, double max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }

public:
    /**
     * Creates a new SlewRateLimiter with the given positive and negative rate limits and initial
     * value.
     *
     * @param positiveRateLimit The rate-of-change limit in the positive direction, in units per
     *     second. This is expected to be positive.
     * @param negativeRateLimit The rate-of-change limit in the negative direction, in units per
     *     second. This is expected to be negative.
     * @param initialValue The initial value of the input.
     */
    SlewRateLimiter(double positiveRateLimit, double negativeRateLimit, double initialValue)
        : m_positiveRateLimit(positiveRateLimit),
          m_negativeRateLimit(negativeRateLimit),
          m_prevVal(initialValue),
          m_prevTime(micros()) {
    }

    /**
     * Creates a new SlewRateLimiter with the given positive rate limit and negative rate limit of
     * -rateLimit.
     *
     * @param rateLimit The rate-of-change limit, in units per second.
     */
    SlewRateLimiter(double rateLimit)
        : SlewRateLimiter(rateLimit, -rateLimit, 0.0) {
    }

    /**
     * Filters the input to limit its slew rate.
     *
     * @param input The input value whose slew rate is to be limited.
     * @return The filtered value, which will not change faster than the slew rate.
     */
    double calculate(double input) {
        unsigned long currentTime = micros();
        double elapsedTime = (currentTime - m_prevTime) / 1000000.0; // Convert microseconds to seconds

        m_prevVal += clamp(
            input - m_prevVal,
            m_negativeRateLimit * elapsedTime,
            m_positiveRateLimit * elapsedTime
        );

        m_prevTime = currentTime;
        return m_prevVal;
    }

    /**
     * Returns the value last calculated by the SlewRateLimiter.
     *
     * @return The last value.
     */
    double lastValue() {
        return m_prevVal;
    }

    /**
     * Resets the slew rate limiter to the specified value; ignores the rate limit when doing so.
     *
     * @param value The value to reset to.
     */
    void reset(double value) {
        m_prevVal = value;
        m_prevTime = micros();
    }
};

#endif