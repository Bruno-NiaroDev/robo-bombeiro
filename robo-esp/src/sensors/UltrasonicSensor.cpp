#include "sensors/UltrasonicSensor.h"
#include "Pins.h"
#include <Arduino.h>

namespace sensors {

static const unsigned long kTriggerPulseMicros = 10;
static const float kCentimetersPerMicrosecond = 0.0343f;

UltrasonicSensor::UltrasonicSensor()
    : UltrasonicSensor(pins::sensors::ultrasonic::TRIG, pins::sensors::ultrasonic::ECHO) {}

UltrasonicSensor::UltrasonicSensor(int triggerPin, int echoPin)
    : _triggerPin(triggerPin), _echoPin(echoPin) {}

void UltrasonicSensor::begin() {
    pinMode(_triggerPin, OUTPUT);
    pinMode(_echoPin, INPUT);
    digitalWrite(_triggerPin, LOW);

    _state = State::Idle;
    _lastMeasurementMillis = 0;
    _triggerStartedMicros = 0;
    _echoStartedMicros = 0;
    _waitStartedMicros = 0;
    _distanceCm = -1.0f;
    _hasNewData = false;
}

void UltrasonicSensor::update(unsigned long currentMillis) {
    unsigned long currentMicros = micros();

    switch (_state) {
        case State::Idle:
            if (currentMillis - _lastMeasurementMillis >= _measurementIntervalMillis) {
                startMeasurement();
            }
            break;

        case State::TriggerHigh:
            if (currentMicros - _triggerStartedMicros >= kTriggerPulseMicros) {
                digitalWrite(_triggerPin, LOW);
                _waitStartedMicros = currentMicros;
                _state = State::WaitingEchoRise;
            }
            break;

        case State::WaitingEchoRise:
            if (digitalRead(_echoPin) == HIGH) {
                _echoStartedMicros = currentMicros;
                _state = State::WaitingEchoFall;
            } else if (echoTimedOut(currentMicros)) {
                failMeasurement(currentMicros);
            }
            break;

        case State::WaitingEchoFall:
            if (digitalRead(_echoPin) == LOW) {
                finishMeasurement(currentMicros - _echoStartedMicros);
            } else if (echoTimedOut(currentMicros)) {
                failMeasurement(currentMicros);
            }
            break;
    }
}

bool UltrasonicSensor::hasNewData() const {
    return _hasNewData;
}

bool UltrasonicSensor::startMeasurement() {
    if (_state != State::Idle) {
        return false;
    }

    _hasNewData = false;
    _triggerStartedMicros = micros();
    _waitStartedMicros = _triggerStartedMicros;
    digitalWrite(_triggerPin, HIGH);
    _state = State::TriggerHigh;
    return true;
}

void UltrasonicSensor::clearNewData() {
    _hasNewData = false;
}

float UltrasonicSensor::distanceCm() const {
    return _distanceCm;
}

bool UltrasonicSensor::obstacleDetected() const {
    return _distanceCm >= 0.0f && _distanceCm <= _obstacleThresholdCm;
}

bool UltrasonicSensor::isMeasuring() const {
    return _state != State::Idle;
}

void UltrasonicSensor::setObstacleThresholdCm(float thresholdCm) {
    if (thresholdCm > 0.0f) {
        _obstacleThresholdCm = thresholdCm;
    }
}

void UltrasonicSensor::setMeasurementInterval(unsigned long intervalMillis) {
    _measurementIntervalMillis = intervalMillis;
}

void UltrasonicSensor::setMaxEchoTime(unsigned long timeoutMicros) {
    if (timeoutMicros > 0) {
        _maxEchoTimeMicros = timeoutMicros;
    }
}

void UltrasonicSensor::finishMeasurement(unsigned long echoDurationMicros) {
    _distanceCm = (echoDurationMicros * kCentimetersPerMicrosecond) / 2.0f;
    _lastMeasurementMillis = millis();
    _hasNewData = true;
    _state = State::Idle;
}

void UltrasonicSensor::failMeasurement(unsigned long currentMicros) {
    (void)currentMicros;
    _distanceCm = -1.0f;
    _lastMeasurementMillis = millis();
    _hasNewData = true;
    _state = State::Idle;
    digitalWrite(_triggerPin, LOW);
}

bool UltrasonicSensor::echoTimedOut(unsigned long currentMicros) const {
    return currentMicros - _waitStartedMicros >= _maxEchoTimeMicros;
}

} // namespace sensors
