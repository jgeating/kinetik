#ifndef _CLOSEST_ANGLE_STEERING_
#define _CLOSEST_ANGLE_STEERING_

#include <Arduino.h>

/**
 * A class that limits the rate of change of an input value. Useful for implementing voltage,
 * setpoint, and/or output ramps. A slew-rate limit is most appropriate when the quantity being
 * controlled is a velocity or a voltage; when controlling a position, consider using a
 * trapezoidal profile instead.
 */
class ClosestAngleSteering {
private:
    double m_currentAngleDeg;

    /**
     * Calculate the shortest angular difference between two angles in degrees.
     * Handles angle wrapping and treats 180/-180 as the same angle.
     * Returns a value between -90 and 90 degrees.
     *
     * @param currentAngle Current angle in degrees
     * @param targetAngle Target angle in degrees
     * @return Angular difference in degrees, range [-90, 90]
     */
    double angleDifference(double currentAngleDeg, double targetAngleDeg) {
        // Normalize angles to [-180, 180] range
        auto normalizeAngle = [](double angle) -> double {
            angle = fmod(angle, 360.0);
            if (angle > 180.0) {
                angle -= 360.0;
            } else if (angle <= -180.0) {
                angle += 360.0;
            }
            return angle;
        };

        double normCurrent = normalizeAngle(currentAngleDeg);
        double normTarget = normalizeAngle(targetAngleDeg);

        // Calculate the raw difference
        double diff = normTarget - normCurrent;

        // Wrap difference to [-180, 180]
        if (diff > 180.0) {
            diff -= 360.0;
        } else if (diff <= -180.0) {
            diff += 360.0;
        }

        // Handle the special case where difference is exactly 180 or -180
        // In this case, we treat them as the same angle (difference = 0)
        if (abs(diff) == 180.0) {
            diff = 0.0;
        }

        // Clamp to [-90, 90] range as requested
        if (diff > 90.0) {
            diff -= 180.0;
        } else if (diff < -90.0) {
            diff += 180.0;
        }

        return diff;
    }

public:

    ClosestAngleSteering() {
        m_currentAngleDeg = 0;
    }

    double calculate(double targetAngleRad) {
        double targetAngleDegrees = targetAngleRad * 180.0 / PI;
        double angleDiff = angleDifference(m_currentAngleDeg, targetAngleDegrees);
        m_currentAngleDeg += angleDiff;
        return -m_currentAngleDeg * PI / 180.0;
    }
};

#endif