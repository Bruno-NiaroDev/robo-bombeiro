#pragma once

#include "TaskScheduler.h"

namespace core {

class SystemManager {
public:
    void begin();
    void update(unsigned long currentMillis);

private:
    TaskScheduler _scheduler;
};

} // namespace core
