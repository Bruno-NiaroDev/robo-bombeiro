#include "sensors/SensorManager.h"

namespace sensors {

void SensorManager::begin() {
    for (SensorBase* sensor : _sensors) {
        if (sensor) {
            sensor->begin();
        }
    }
}

void SensorManager::update(unsigned long currentMillis) {
    for (SensorBase* sensor : _sensors) {
        if (sensor) {
            sensor->update(currentMillis);
        }
    }
}

void SensorManager::registerSensor(SensorBase* sensor) {
    if (sensor) {
        _sensors.push_back(sensor);
    }
}

} // namespace sensors
