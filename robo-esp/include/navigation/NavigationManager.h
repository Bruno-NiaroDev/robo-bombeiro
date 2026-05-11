#pragma once

#include <vector>
#include "GridMap.h"
#include "PathPlanner.h"

namespace navigation {

class NavigationManager {
public:
    void begin();
    void update(unsigned long currentMillis);

    bool requestPath(uint8_t goalX, uint8_t goalY);
    bool requestPath(uint8_t robotX, uint8_t robotY, uint8_t goalX, uint8_t goalY);
    bool requestPath(Coordinate robotPosition, Coordinate goal);

    bool isNavigationActive() const;
    const std::vector<Coordinate>& currentPath() const;
    const char* lastError() const;
    GridMap& gridMap();
    const GridMap& gridMap() const;

private:
    GridMap _gridMap;
    PathPlanner _planner;
    bool _navigationActive = false;
};

} // namespace navigation
