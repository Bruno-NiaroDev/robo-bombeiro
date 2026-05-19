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
        analogSetPinAttenuation(_flameSensorPin, ADC_11db);

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

        _ambientBaseline = 0;

        _ambientBaselineReady = false;

        _baselineAccumulator = 0;

        _baselineSamples = 0;

        _detectionStartedAt = 0;

        _sweepStartedAt = 0;

        _candidateStartedAt = 0;

        _fireDirection =
            FireDirection::None;

        _detectionEnabled = false;

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

        updateFireReading(currentMillis);
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

    void FireScanner::setFireThreshold(
        uint16_t threshold)
    {

        if (threshold <= 4095)
        {

            _fireThreshold = threshold;
        }
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

    void FireScanner::enableDetection(bool enabled)
    {

        if (enabled == _detectionEnabled)
        {

            return;
        }

        if (enabled && !_detectionEnabled)
        {

            resetDetection();

            _detectionStartedAt =
                millis();
        }

        if (!enabled)
        {

            resetDetection();
        }

        _detectionEnabled = enabled;

        Serial.printf(
            "FIRE DETECTION: %s\n",
            enabled ? "ENABLED" : "DISABLED");
    }

    void FireScanner::enableSweep(bool enabled)
    {

        if (enabled && !_sweepEnabled)
        {

            _sweepStartedAt =
                millis();
        }

        _sweepEnabled = enabled;

        Serial.printf(
            "FIRE SWEEP: %s\n",
            enabled ? "ENABLED" : "DISABLED");
    }

    void FireScanner::center()
    {

        _currentAngle =
            clampAngle(_centerAngle);

        _fireAngle =
            _currentAngle;

        _fireDirection =
            FireDirection::None;

        _fireDetected = false;

        _candidateStartedAt = 0;

        _sweepIncreasing = true;

        writeServoAngle(_currentAngle);

        Serial.printf(
            "FIRE SERVO CENTERED: angle=%u\n",
            _currentAngle);
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

    void FireScanner::updateFireReading(
        unsigned long currentMillis)
    {

        int rawValue =
            analogRead(_flameSensorPin);

        if (rawValue < 0)
        {

            return;
        }

        uint16_t raw =
            static_cast<uint16_t>(rawValue);

        bool calibrating =
            !_detectionEnabled ||
            currentMillis - _detectionStartedAt < _calibrationDurationMillis;

        if (!_detectionEnabled)
        {

            if (_fireDetected)
            {

                _hasNewData = true;
            }

            _fireDetected = false;
            _fireDirection = FireDirection::None;
            _candidateStartedAt = 0;
            resetBaseline();
            return;
        }

        if (calibrating || !_ambientBaselineReady)
        {

            _baselineAccumulator += raw;
            ++_baselineSamples;
            if (raw < _calibrationMin) {
                _calibrationMin = raw;
            }
            if (raw > _calibrationMax) {
                _calibrationMax = raw;
            }

            if (_baselineSamples > 0)
            {

                _ambientBaseline =
                    static_cast<uint16_t>(
                        _baselineAccumulator /
                        _baselineSamples);

                _ambientBaselineReady = true;

                uint16_t noiseRange =
                    static_cast<uint16_t>(_calibrationMax - _calibrationMin);
                uint16_t noiseThreshold =
                    static_cast<uint16_t>((noiseRange * 3) + 40);
                _effectiveFireThreshold =
                    noiseThreshold > _fireThreshold ? noiseThreshold : _fireThreshold;
            }
        }

        uint16_t distanceFromBaseline =
            raw > _ambientBaseline
                ? static_cast<uint16_t>(raw - _ambientBaseline)
                : static_cast<uint16_t>(_ambientBaseline - raw);

        Serial.printf(
            "FLAME RAW=%u BASE=%u DIFF=%u THRESH=%u NOISE=%u DET=%s CAL=%s\n",
            raw,
            _ambientBaseline,
            distanceFromBaseline,
            _effectiveFireThreshold,
            static_cast<uint16_t>(_calibrationMax - _calibrationMin),
            _detectionEnabled ? "true" : "false",
            calibrating ? "true" : "false");

        if (calibrating)
        {

            _fireDetected = false;
            _fireDirection = FireDirection::None;
            _candidateStartedAt = 0;
            return;
        }

        bool sampleDetected =
            _ambientBaselineReady &&
            distanceFromBaseline >= _effectiveFireThreshold;

        if (sampleDetected)
        {

            if (_candidateStartedAt == 0)
            {

                _candidateStartedAt = currentMillis;
            }
        }
        else
        {

            _candidateStartedAt = 0;
        }

        bool detected =
            _candidateStartedAt != 0 &&
            currentMillis - _candidateStartedAt >= _confirmationDurationMillis;

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

    void FireScanner::resetDetection()
    {

        _fireDetected = false;
        _hasNewData = false;
        _fireDirection = FireDirection::None;
        _candidateStartedAt = 0;
        resetBaseline();
    }

    void FireScanner::resetBaseline()
    {

        _ambientBaseline = 0;
        _baselineAccumulator = 0;
        _baselineSamples = 0;
        _calibrationMin = 4095;
        _calibrationMax = 0;
        _effectiveFireThreshold = _fireThreshold;
        _ambientBaselineReady = false;
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
