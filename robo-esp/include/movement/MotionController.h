#pragma once

#include "TrajectoryPlanner.h"

namespace movement {

class MotionController {
public:
    void begin();
    void update(unsigned long currentMillis);
    void moveTo(uint8_t x, uint8_t y);
    bool isMoving() const;

private:
    TrajectoryPlanner _planner;
};

} // namespace movement
