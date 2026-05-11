#pragma once

#include <stdint.h>
#include "SensorBase.h"

namespace sensors {

enum class FireDirection {
    None,
    Left,
    Center,
    Right
};

class FireScanner : public SensorBase {
public:
    FireScanner();
    FireScanner(int servoPin, int flameSensorPin);

    void begin() override;
    void update(unsigned long currentMillis) override;
    bool hasNewData() const override;

    void clearNewData();

    bool fireDetected() const;
    FireDirection fireDirection() const;
    uint8_t fireAngle() const;
    uint8_t currentAngle() const;

    void setSweepRange(uint8_t minAngle, uint8_t maxAngle);
    void setCenterAngle(uint8_t centerAngle);
    void setStepInterval(unsigned long intervalMillis);
    void setStepDegrees(uint8_t stepDegrees);
    void setSensorActiveLow(bool activeLow);
    void setServoPulseRange(uint16_t minPulseMicros, uint16_t maxPulseMicros);

private:
    void updateSweep(unsigned long currentMillis);
    void updateFireReading();
    void writeServoAngle(uint8_t angle);
    uint16_t pulseForAngle(uint8_t angle) const;
    FireDirection directionForAngle(uint8_t angle) const;
    uint8_t clampAngle(uint8_t angle) const;

    int _servoPin;
    int _flameSensorPin;
    uint8_t _pwmChannel = 2;
    uint8_t _currentAngle = 90;
    uint8_t _minAngle = 20;
    uint8_t _maxAngle = 160;
    uint8_t _centerAngle = 90;
    uint8_t _stepDegrees = 1;
    uint8_t _fireAngle = 90;
    uint16_t _minPulseMicros = 500;
    uint16_t _maxPulseMicros = 2400;
    unsigned long _lastStepMillis = 0;
    unsigned long _stepIntervalMillis = 20;
    bool _sweepIncreasing = true;
    bool _sensorActiveLow = true;
    bool _fireDetected = false;
    bool _hasNewData = false;
    FireDirection _fireDirection = FireDirection::None;
};

} // namespace sensors
