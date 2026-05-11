#pragma once

namespace core {

class TaskScheduler {
public:
    void begin();
    void update(unsigned long currentMillis);
    void registerTask(void (*task)(unsigned long));
};

} // namespace core
