#pragma once

namespace drivers {

class SensorDriver {
public:
    void begin();
    void read();
    bool hasNewData() const;
    float value() const;
    void update(unsigned long currentMillis);
};

} // namespace drivers
