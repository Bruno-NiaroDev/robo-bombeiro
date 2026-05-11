#pragma once

#include <vector>
#include "SensorBase.h"

namespace sensors {

class SensorManager {
public:
    void begin();
    void update(unsigned long currentMillis);
    void registerSensor(SensorBase* sensor);

private:
    std::vector<SensorBase*> _sensors;
};

} // namespace sensors
