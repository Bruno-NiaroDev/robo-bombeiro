#include "navigation/PathPlanner.h"
#include <Arduino.h>
#include <stdio.h>
#include <queue>

namespace navigation {

void PathPlanner::begin() {
    _path.clear();
}

void PathPlanner::setGridMap(const GridMap& gridMap) {
    _gridMap = &gridMap;
}

bool PathPlanner::findPath(Coordinate start, Coordinate goal) {
    _path.clear();
    setLastError("");

    Serial.println("BFS START");
    Serial.printf("BFS ORIGIN: (%u,%u)\n", start.x, start.y);
    Serial.printf("BFS DESTINATION: (%u,%u)\n", goal.x, goal.y);

    if (!_gridMap) {
        return fail("BFS failed: grid map not configured");
    }

    if (!_gridMap->isValidPosition(start.x, start.y)) {
        snprintf(_lastError, sizeof(_lastError), "BFS failed: invalid origin coordinate (%u,%u)", start.x, start.y);
        Serial.println(_lastError);
        return false;
    }

    if (!_gridMap->isValidPosition(goal.x, goal.y)) {
        snprintf(_lastError, sizeof(_lastError), "BFS failed: invalid destination coordinate (%u,%u)", goal.x, goal.y);
        Serial.println(_lastError);
        return false;
    }

    if (!canVisit(start.x, start.y, start, goal)) {
        snprintf(_lastError, sizeof(_lastError), "BFS failed: origin blocked (%u,%u)", start.x, start.y);
        Serial.println(_lastError);
        return false;
    }

    if (!canVisit(goal.x, goal.y, start, goal)) {
        snprintf(_lastError, sizeof(_lastError), "BFS failed: destination blocked (%u,%u) fixed=%s dynamic=%s",
                 goal.x,
                 goal.y,
                 _gridMap->isFixedBlocked(goal.x, goal.y) ? "true" : "false",
                 _gridMap->hasDynamicObstacle(goal.x, goal.y) ? "true" : "false");
        Serial.println(_lastError);
        return false;
    }

    bool visited[GridMap::Height][GridMap::Width] = {};
    Coordinate parents[GridMap::Height][GridMap::Width] = {};
    std::queue<Coordinate> queue;
    uint8_t visitedCount = 0;

    for (uint8_t y = 0; y < GridMap::Height; ++y) {
        for (uint8_t x = 0; x < GridMap::Width; ++x) {
            parents[y][x] = {GridMap::Width, GridMap::Height};
        }
    }

    visited[start.y][start.x] = true;
    queue.push(start);
    ++visitedCount;
    Serial.printf("BFS VISITED NODE: (%u,%u)\n", start.x, start.y);

    const int8_t directions[4][2] = {
        {0, -1},
        {1, 0},
        {0, 1},
        {-1, 0}
    };

    while (!queue.empty()) {
        Coordinate current = queue.front();
        queue.pop();

        if (current.x == goal.x && current.y == goal.y) {
            reconstructPath(start, goal, parents);
            Serial.printf("BFS VISITED COUNT: %u\n", visitedCount);
            Serial.printf("BFS PATH SIZE: %u\n", static_cast<unsigned>(_path.size()));
            if (_path.empty()) {
                return fail("BFS failed: empty path after reconstruction");
            }
            Serial.println("BFS SUCCESS");
            return true;
        }

        for (uint8_t i = 0; i < 4; ++i) {
            int16_t nextX = static_cast<int16_t>(current.x) + directions[i][0];
            int16_t nextY = static_cast<int16_t>(current.y) + directions[i][1];

            if (nextX < 0 || nextY < 0) {
                continue;
            }

            uint8_t x = static_cast<uint8_t>(nextX);
            uint8_t y = static_cast<uint8_t>(nextY);

            if (!_gridMap->isValidPosition(x, y) ||
                visited[y][x] ||
                !canVisit(x, y, start, goal)) {
                continue;
            }

            visited[y][x] = true;
            parents[y][x] = current;
            queue.push({x, y});
            ++visitedCount;
            Serial.printf("BFS VISITED NODE: (%u,%u)\n", x, y);
        }
    }

    Serial.printf("BFS VISITED COUNT: %u\n", visitedCount);
    return fail("BFS failed: no valid path exists between origin and destination");
}

bool PathPlanner::findPath(uint8_t startX, uint8_t startY, uint8_t goalX, uint8_t goalY) {
    return findPath({startX, startY}, {goalX, goalY});
}

void PathPlanner::planPath(uint8_t startX, uint8_t startY, uint8_t goalX, uint8_t goalY) {
    findPath(startX, startY, goalX, goalY);
}

const std::vector<Coordinate>& PathPlanner::currentPath() const {
    return _path;
}

const char* PathPlanner::lastError() const {
    return _lastError;
}

void PathPlanner::update(unsigned long currentMillis) {}

bool PathPlanner::canVisit(uint8_t x, uint8_t y, Coordinate start, Coordinate goal) const {
    if (x == start.x && y == start.y) {
        return !_gridMap->isFixedBlocked(x, y);
    }

    if (x == goal.x && y == goal.y) {
        return _gridMap->isCellFree(x, y) || (x == start.x && y == start.y);
    }

    return _gridMap->isCellFree(x, y);
}

void PathPlanner::reconstructPath(Coordinate start, Coordinate goal, Coordinate parents[GridMap::Height][GridMap::Width]) {
    std::vector<Coordinate> reversedPath;
    Coordinate current = goal;

    reversedPath.push_back(current);

    while (!(current.x == start.x && current.y == start.y)) {
        current = parents[current.y][current.x];
        reversedPath.push_back(current);
    }

    _path.clear();
    _path.reserve(reversedPath.size());

    for (uint8_t i = static_cast<uint8_t>(reversedPath.size()); i > 0; --i) {
        _path.push_back(reversedPath[i - 1]);
    }
}

bool PathPlanner::fail(const char* message) {
    setLastError(message);
    Serial.println(_lastError);
    return false;
}

void PathPlanner::setLastError(const char* message) {
    if (!message) {
        _lastError[0] = '\0';
        return;
    }

    snprintf(_lastError, sizeof(_lastError), "%s", message);
}

} // namespace navigation
