#pragma once

#include <stdint.h>
#include "SensorBase.h"

namespace sensors {

class UltrasonicSensor : public SensorBase {
public:
    UltrasonicSensor();
    UltrasonicSensor(int triggerPin, int echoPin);

    void begin() override;
    void update(unsigned long currentMillis) override;
    bool hasNewData() const override;

    bool startMeasurement();
    void clearNewData();

    float distanceCm() const;
    bool obstacleDetected() const;
    bool isMeasuring() const;

    void setObstacleThresholdCm(float thresholdCm);
    void setMeasurementInterval(unsigned long intervalMillis);
    void setMaxEchoTime(unsigned long timeoutMicros);

private:
    enum class State {
        Idle,
        TriggerHigh,
        WaitingEchoRise,
        WaitingEchoFall
    };

    void finishMeasurement(unsigned long echoDurationMicros);
    void failMeasurement(unsigned long currentMicros);
    bool echoTimedOut(unsigned long currentMicros) const;

    int _triggerPin;
    int _echoPin;
    State _state = State::Idle;
    unsigned long _lastMeasurementMillis = 0;
    unsigned long _triggerStartedMicros = 0;
    unsigned long _echoStartedMicros = 0;
    unsigned long _waitStartedMicros = 0;
    unsigned long _measurementIntervalMillis = 60;
    unsigned long _maxEchoTimeMicros = 30000;
    float _distanceCm = -1.0f;
    float _obstacleThresholdCm = 20.0f;
    bool _hasNewData = false;
};

} // namespace sensors
