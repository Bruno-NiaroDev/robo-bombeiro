#pragma once

#include <stdint.h>

namespace actuators {

class PumpController {
public:
    PumpController();
    explicit PumpController(int relayPin);

    void begin();
    void update(unsigned long currentMillis);

    void turnOn();
    void turnOff();
    bool runFor(unsigned long durationMillis, unsigned long currentMillis);

    bool isOn() const;
    bool isTimedRunActive() const;

    void setRelayActiveHigh(bool activeHigh);
    void setMaxRunTime(unsigned long maxRunTimeMillis);

private:
    void applyRelayState();
    unsigned long safeDuration(unsigned long durationMillis) const;

    int _relayPin;
    bool _relayActiveHigh = true;
    bool _isOn = false;
    bool _timedRunActive = false;
    unsigned long _timedRunStartedAt = 0;
    unsigned long _timedRunDuration = 0;
    unsigned long _maxRunTimeMillis = 5000;
};

} // namespace actuators
