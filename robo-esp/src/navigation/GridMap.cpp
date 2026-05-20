#include "navigation/GridMap.h"
#include <Arduino.h>

namespace navigation {

const GridCell GridMap::Home = {0, 0};

void GridMap::begin() {
    clear(_fixedBlocked);
    clear(_dynamicObstacles);
    loadFixedBlockedCells();
}

bool GridMap::isValidPosition(uint8_t x, uint8_t y) const {
    return x < Width && y < Height;
}

bool GridMap::isCellFree(uint8_t x, uint8_t y) const {
    return isValidPosition(x, y) && !isCellOccupied(x, y);
}

bool GridMap::isCellOccupied(uint8_t x, uint8_t y) const {
    if (!isValidPosition(x, y)) {
        return true;
    }

    return _fixedBlocked[y][x] || _dynamicObstacles[y][x];
}

bool GridMap::isFixedBlocked(uint8_t x, uint8_t y) const {
    return isValidPosition(x, y) && _fixedBlocked[y][x];
}

bool GridMap::hasDynamicObstacle(uint8_t x, uint8_t y) const {
    return isValidPosition(x, y) && _dynamicObstacles[y][x];
}

void GridMap::debugPrintGrid(uint8_t robotX, uint8_t robotY, uint8_t targetX, uint8_t targetY, bool hasTarget) const {
    Serial.println("----- GRID DEBUG -----");
    Serial.println("Legenda: H=HOME R=ROBO X=BLOQUEADO FIXO O=OBSTACULO DINAMICO F=DESTINO .=LIVRE");

    for (uint8_t y = 0; y < Height; ++y) {
        for (uint8_t x = 0; x < Width; ++x) {
            char marker = '.';

            if (x == Home.x && y == Home.y) {
                marker = 'H';
            }

            if (_fixedBlocked[y][x]) {
                marker = 'X';
            } else if (_dynamicObstacles[y][x]) {
                marker = 'O';
            }

            if (hasTarget && x == targetX && y == targetY) {
                marker = 'F';
            }

            if (x == robotX && y == robotY) {
                marker = 'R';
            }

            Serial.print(marker);
            Serial.print(' ');
        }
        Serial.println();
    }

    Serial.println("----------------------");
}

void GridMap::setCellOccupied(uint8_t x, uint8_t y, bool occupied) {
    setDynamicObstacle(x, y, occupied);
}

bool GridMap::setDynamicObstacle(uint8_t x, uint8_t y, bool occupied) {
    if (!isValidPosition(x, y) || isFixedBlocked(x, y)) {
        return false;
    }

    _dynamicObstacles[y][x] = occupied;
    return true;
}

void GridMap::resetDynamicObstacles() {
    clear(_dynamicObstacles);
}

void GridMap::loadFixedBlockedCells() {
    // Obstaculos fixos da arena 4x4
    _fixedBlocked[1][1] = true;
    _fixedBlocked[2][1] = true;
    _fixedBlocked[2][1] = true;
    _fixedBlocked[2][2] = true;
}

void GridMap::clear(bool cells[Height][Width]) {
    for (uint8_t y = 0; y < Height; ++y) {
        for (uint8_t x = 0; x < Width; ++x) {
            cells[y][x] = false;
        }
    }
}

} // namespace navigation
