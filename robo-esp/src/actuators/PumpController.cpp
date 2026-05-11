#include "actuators/PumpController.h"
#include "Pins.h"
#include <Arduino.h>

namespace actuators {

PumpController::PumpController()
    : PumpController(pins::pump::RELAY) {}

PumpController::PumpController(int relayPin)
    : _relayPin(relayPin) {}

void PumpController::begin() {
    pinMode(_relayPin, OUTPUT);
    turnOff();
}

void PumpController::update(unsigned long currentMillis) {
    if (_timedRunActive && currentMillis - _timedRunStartedAt >= _timedRunDuration) {
        turnOff();
    }
}

void PumpController::turnOn() {
    _isOn = true;
    _timedRunActive = false;
    applyRelayState();
}

void PumpController::turnOff() {
    _isOn = false;
    _timedRunActive = false;
    _timedRunStartedAt = 0;
    _timedRunDuration = 0;
    applyRelayState();
}

bool PumpController::runFor(unsigned long durationMillis, unsigned long currentMillis) {
    unsigned long duration = safeDuration(durationMillis);

    if (duration == 0) {
        turnOff();
        return false;
    }

    _isOn = true;
    _timedRunActive = true;
    _timedRunStartedAt = currentMillis;
    _timedRunDuration = duration;
    applyRelayState();
    return true;
}

bool PumpController::isOn() const {
    return _isOn;
}

bool PumpController::isTimedRunActive() const {
    return _timedRunActive;
}

void PumpController::setRelayActiveHigh(bool activeHigh) {
    _relayActiveHigh = activeHigh;
    applyRelayState();
}

void PumpController::setMaxRunTime(unsigned long maxRunTimeMillis) {
    _maxRunTimeMillis = maxRunTimeMillis;

    if (_timedRunActive && _timedRunDuration > _maxRunTimeMillis) {
        _timedRunDuration = _maxRunTimeMillis;
    }
}

void PumpController::applyRelayState() {
    bool relayLevel = _relayActiveHigh ? _isOn : !_isOn;
    digitalWrite(_relayPin, relayLevel ? HIGH : LOW);
}

unsigned long PumpController::safeDuration(unsigned long durationMillis) const {
    if (_maxRunTimeMillis == 0) {
        return durationMillis;
    }

    return durationMillis > _maxRunTimeMillis ? _maxRunTimeMillis : durationMillis;
}

} // namespace actuators
