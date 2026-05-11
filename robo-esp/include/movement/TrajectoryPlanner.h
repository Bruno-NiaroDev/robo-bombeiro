#pragma once

#include <stdint.h>
#include <vector>

namespace movement {

struct TrajectoryPoint {
    uint8_t x;
    uint8_t y;
};

class TrajectoryPlanner {
public:
    void begin();
    void buildTrajectory(uint8_t startX, uint8_t startY, uint8_t goalX, uint8_t goalY);
    const std::vector<TrajectoryPoint>& trajectory() const;
    void update(unsigned long currentMillis);

private:
    std::vector<TrajectoryPoint> _trajectory;
};

} // namespace movement
