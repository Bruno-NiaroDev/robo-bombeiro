#pragma once

#include <stdint.h>
#include <vector>
#include "GridMap.h"

namespace navigation {

struct Coordinate {
    uint8_t x;
    uint8_t y;
};

using PathPoint = Coordinate;

class PathPlanner {
public:
    void begin();

    void setGridMap(const GridMap& gridMap);

    bool findPath(Coordinate start, Coordinate goal);
    bool findPath(uint8_t startX, uint8_t startY, uint8_t goalX, uint8_t goalY);

    void planPath(uint8_t startX, uint8_t startY, uint8_t goalX, uint8_t goalY);
    const std::vector<Coordinate>& currentPath() const;
    const char* lastError() const;
    void update(unsigned long currentMillis);

private:
    bool canVisit(uint8_t x, uint8_t y, Coordinate start, Coordinate goal) const;
    void reconstructPath(Coordinate start, Coordinate goal, Coordinate parents[GridMap::Height][GridMap::Width]);
    bool fail(const char* message);
    void setLastError(const char* message);

    const GridMap* _gridMap = nullptr;
    std::vector<Coordinate> _path;
    char _lastError[128] = "";
};

} // namespace navigation
