#include "navigation/NavigationManager.h"

namespace navigation {

void NavigationManager::begin() {
    _gridMap.begin();
    _planner.begin();
    _planner.setGridMap(_gridMap);
    _navigationActive = false;
}

void NavigationManager::update(unsigned long currentMillis) {
    _planner.update(currentMillis);
}

bool NavigationManager::requestPath(uint8_t goalX, uint8_t goalY) {
    return requestPath(GridMap::Home.x, GridMap::Home.y, goalX, goalY);
}

bool NavigationManager::requestPath(uint8_t robotX, uint8_t robotY, uint8_t goalX, uint8_t goalY) {
    return requestPath({robotX, robotY}, {goalX, goalY});
}

bool NavigationManager::requestPath(Coordinate robotPosition, Coordinate goal) {
    _navigationActive = _planner.findPath(robotPosition, goal);
    return _navigationActive;
}

bool NavigationManager::isNavigationActive() const {
    return _navigationActive;
}

const std::vector<Coordinate>& NavigationManager::currentPath() const {
    return _planner.currentPath();
}

const char* NavigationManager::lastError() const {
    return _planner.lastError();
}

GridMap& NavigationManager::gridMap() {
    return _gridMap;
}

const GridMap& NavigationManager::gridMap() const {
    return _gridMap;
}

} // namespace navigation
