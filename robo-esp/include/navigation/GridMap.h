#pragma once

#include <stdint.h>

namespace navigation {

struct GridCell {
    uint8_t x;
    uint8_t y;
};

class GridMap {
public:
    static const uint8_t Width = 4;
    static const uint8_t Height = 4;
    static const GridCell Home;

    void begin();

    bool isValidPosition(uint8_t x, uint8_t y) const;
    bool isCellFree(uint8_t x, uint8_t y) const;
    bool isCellOccupied(uint8_t x, uint8_t y) const;
    bool isFixedBlocked(uint8_t x, uint8_t y) const;
    bool hasDynamicObstacle(uint8_t x, uint8_t y) const;
    void debugPrintGrid(uint8_t robotX, uint8_t robotY, uint8_t targetX, uint8_t targetY, bool hasTarget) const;

    void setCellOccupied(uint8_t x, uint8_t y, bool occupied);
    bool setDynamicObstacle(uint8_t x, uint8_t y, bool occupied);
    void resetDynamicObstacles();

private:
    void loadFixedBlockedCells();
    void clear(bool cells[Height][Width]);

    bool _fixedBlocked[Height][Width] = {};
    bool _dynamicObstacles[Height][Width] = {};
};

} // namespace navigation
