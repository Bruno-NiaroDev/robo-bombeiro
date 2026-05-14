#include "sensors/FireScanner.h"
#include "Pins.h"

#include <Arduino.h>

namespace sensors
{

    static const uint16_t kServoFrequencyHz = 50;
    static const uint8_t kServoPwmResolutionBits = 16;
    static const uint32_t kServoPeriodMicros = 20000;
    static const uint32_t kMaxDuty =
        (1UL << kServoPwmResolutionBits) - 1;

    FireScanner::FireScanner()
        : FireScanner(
              pins::servo::HORIZONTAL,
              pins::sensors::FLAME)
    {
    }

    FireScanner::FireScanner(
        int servoPin,
        int flameSensorPin)
        : _servoPin(servoPin),
          _flameSensorPin(flameSensorPin)
    {
    }

    void FireScanner::begin()
    {

        pinMode(_flameSensorPin, INPUT);

        analogReadResolution(12);

        ledcSetup(
            _pwmChannel,
            kServoFrequencyHz,
            kServoPwmResolutionBits);

        ledcAttachPin(
            _servoPin,
            _pwmChannel);

        _currentAngle =
            clampAngle(_centerAngle);

        _fireAngle =
            _currentAngle;

        _lastStepMillis = 0;

        _sweepIncreasing = true;

        _fireDetected = false;

        _hasNewData = false;

        _fireDirection =
            FireDirection::None;

        _sweepEnabled = false;

        writeServoAngle(_currentAngle);

        Serial.println(
            "FIRE SCANNER READY");
    }

    void FireScanner::update(
        unsigned long currentMillis)
    {

        if (_sweepEnabled)
        {

            updateSweep(currentMillis);
        }

        updateFireReading();
    }

    bool FireScanner::hasNewData() const
    {

        return _hasNewData;
    }

    void FireScanner::clearNewData()
    {

        _hasNewData = false;
    }

    bool FireScanner::fireDetected() const
    {

        return _fireDetected;
    }

    FireDirection FireScanner::fireDirection() const
    {

        return _fireDirection;
    }

    uint8_t FireScanner::fireAngle() const
    {

        return _fireAngle;
    }

    uint8_t FireScanner::currentAngle() const
    {

        return _currentAngle;
    }

    void FireScanner::setSweepRange(
        uint8_t minAngle,
        uint8_t maxAngle)
    {

        if (
            minAngle >= maxAngle ||
            maxAngle > 180)
        {

            return;
        }

        _minAngle = minAngle;
        _maxAngle = maxAngle;

        _currentAngle =
            clampAngle(_currentAngle);

        writeServoAngle(_currentAngle);
    }

    void FireScanner::setCenterAngle(
        uint8_t centerAngle)
    {

        if (centerAngle <= 180)
        {

            _centerAngle = centerAngle;
        }
    }

    void FireScanner::setStepInterval(
        unsigned long intervalMillis)
    {

        _stepIntervalMillis =
            intervalMillis;
    }

    void FireScanner::setStepDegrees(
        uint8_t stepDegrees)
    {

        if (stepDegrees > 0)
        {

            _stepDegrees = stepDegrees;
        }
    }

    void FireScanner::setSensorActiveLow(
        bool activeLow)
    {

        _sensorActiveLow = activeLow;
    }

    void FireScanner::setServoPulseRange(
        uint16_t minPulseMicros,
        uint16_t maxPulseMicros)
    {

        if (minPulseMicros < maxPulseMicros)
        {

            _minPulseMicros =
                minPulseMicros;

            _maxPulseMicros =
                maxPulseMicros;

            writeServoAngle(_currentAngle);
        }
    }

    void FireScanner::enableSweep(bool enabled)
    {

        _sweepEnabled = enabled;

        Serial.printf(
            "FIRE SWEEP: %s\n",
            enabled ? "ENABLED" : "DISABLED");
    }

    bool FireScanner::sweepEnabled() const
    {

        return _sweepEnabled;
    }

    void FireScanner::updateSweep(
        unsigned long currentMillis)
    {

        if (
            currentMillis - _lastStepMillis <
            _stepIntervalMillis)
        {

            return;
        }

        _lastStepMillis = currentMillis;

        if (_sweepIncreasing)
        {

            if (
                _currentAngle + _stepDegrees >=
                _maxAngle)
            {

                _currentAngle = _maxAngle;

                _sweepIncreasing = false;
            }
            else
            {

                _currentAngle += _stepDegrees;
            }
        }
        else
        {

            if (
                _currentAngle <=
                _minAngle + _stepDegrees)
            {

                _currentAngle = _minAngle;

                _sweepIncreasing = true;
            }
            else
            {

                _currentAngle -= _stepDegrees;
            }
        }

        writeServoAngle(_currentAngle);
    }

    void FireScanner::updateFireReading()
    {

        int rawValue =
            analogRead(_flameSensorPin);

        Serial.printf(
            "FLAME ANALOG = %d\n",
            rawValue);

        /*
            Ajuste conforme necessário.

            Menor valor:
            menos sensível

            Maior valor:
            mais sensível
        */
        const int fireThreshold = 1500;

        bool detected =
            rawValue < fireThreshold;

        FireDirection previousDirection =
            _fireDirection;

        if (detected)
        {

            _fireAngle =
                _currentAngle;

            _fireDirection =
                directionForAngle(
                    _fireAngle);
        }
        else
        {

            _fireDirection =
                FireDirection::None;
        }

        if (
            detected != _fireDetected ||
            _fireDirection != previousDirection)
        {

            _hasNewData = true;
        }

        _fireDetected = detected;
    }

    void FireScanner::writeServoAngle(
        uint8_t angle)
    {

        uint16_t pulseMicros =
            pulseForAngle(angle);

        uint32_t duty =
            (static_cast<uint32_t>(
                 pulseMicros) *
             kMaxDuty) /
            kServoPeriodMicros;

        ledcWrite(
            _pwmChannel,
            duty);
    }

    uint16_t FireScanner::pulseForAngle(
        uint8_t angle) const
    {

        uint32_t pulseRange =
            _maxPulseMicros -
            _minPulseMicros;

        return _minPulseMicros +
               ((pulseRange * angle) / 180);
    }

    FireDirection FireScanner::directionForAngle(
        uint8_t angle) const
    {

        const uint8_t centerWindow = 15;

        if (
            angle + centerWindow <
            _centerAngle)
        {

            return FireDirection::Left;
        }

        if (
            angle >
            _centerAngle + centerWindow)
        {

            return FireDirection::Right;
        }

        return FireDirection::Center;
    }

    uint8_t FireScanner::clampAngle(
        uint8_t angle) const
    {

        if (angle < _minAngle)
        {

            return _minAngle;
        }

        if (angle > _maxAngle)
        {

            return _maxAngle;
        }

        return angle;
    }

} // namespace sensors