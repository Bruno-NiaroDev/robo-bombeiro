#pragma once

#include <stdint.h>

namespace sensors {

class SensorBase {
public:
    virtual ~SensorBase() = default;
    virtual void begin() = 0;
    virtual void update(unsigned long currentMillis) = 0;
    virtual bool hasNewData() const = 0;
};

} // namespace sensors
