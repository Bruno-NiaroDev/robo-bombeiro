#include "robot/Robot.h"
#include <Arduino.h>
#include <stdio.h>

namespace robot {

void Robot::begin() {
    _navigation.begin();
    _state = RobotState();
    _state.mode = RobotMode::Autonomous;
    transitionTo(AutonomousState::IDLE, 0);
}

void Robot::update(unsigned long currentMillis) {
    _navigation.update(currentMillis);

    switch (_state.autonomousState) {
        case AutonomousState::IDLE:
            break;
        case AutonomousState::RECEIVING_TARGET:
            handleReceivingTarget(currentMillis);
            break;
        case AutonomousState::CALCULATING_ROUTE:
            handleCalculatingRoute(currentMillis);
            break;
        case AutonomousState::MOVING:
            handleMoving(currentMillis);
            break;
        case AutonomousState::AVOIDING_OBSTACLE:
            handleAvoidingObstacle(currentMillis);
            break;
        case AutonomousState::SEARCHING_FIRE:
            handleSearchingFire(currentMillis);
            break;
        case AutonomousState::EXTINGUISHING_FIRE:
            handleExtinguishingFire(currentMillis);
            break;
        case AutonomousState::RETURNING_HOME:
            handleReturningHome(currentMillis);
            break;
        case AutonomousState::ERROR:
            break;
    }
}

bool Robot::setTarget(uint8_t x, uint8_t y, unsigned long currentMillis) {
    Serial.printf("TARGET RECEIVED: (%u,%u)\n", x, y);

    if (!_navigation.gridMap().isValidPosition(x, y)) {
        char error[128];
        snprintf(error, sizeof(error), "Invalid target: coordinate outside grid (%u,%u)", x, y);
        enterError(error);
        return false;
    }

    if (!_navigation.gridMap().isCellFree(x, y)) {
        char error[128];
        snprintf(error, sizeof(error), "Invalid target: destination blocked (%u,%u) fixed=%s dynamic=%s",
            x,
            y,
            _navigation.gridMap().isFixedBlocked(x, y) ? "true" : "false",
            _navigation.gridMap().hasDynamicObstacle(x, y) ? "true" : "false");
        enterError(error);
        return false;
    }

    if (!_navigation.gridMap().isValidPosition(_state.x, _state.y) ||
        !_navigation.gridMap().isCellFree(_state.x, _state.y)) {
        char error[128];
        snprintf(error, sizeof(error), "Invalid robot position before target: (%u,%u)", _state.x, _state.y);
        enterError(error);
        return false;
    }

    _state.targetX = x;
    _state.targetY = y;
    _state.hasTarget = true;
    _state.routeReady = false;
    _state.returningHome = false;
    _extinguishingComplete = false;
    _routeRecalculationPending = false;
    setLastError("");
    Serial.println("GRID AFTER TARGET RECEIVED");
    _navigation.gridMap().debugPrintGrid(_state.x, _state.y, _state.targetX, _state.targetY, _state.hasTarget);
    transitionTo(AutonomousState::RECEIVING_TARGET, currentMillis);
    return true;
}

void Robot::setPosition(uint8_t x, uint8_t y) {
    if (!_navigation.gridMap().isValidPosition(x, y)) {
        Serial.printf("POSITION REJECTED: outside grid (%u,%u)\n", x, y);
        return;
    }

    if (!_navigation.gridMap().isCellFree(x, y)) {
        Serial.printf("POSITION REJECTED: blocked cell (%u,%u)\n", x, y);
        return;
    }

    _state.x = x;
    _state.y = y;
    Serial.printf("ROBOT POSITION SET: (%u,%u)\n", x, y);
}

void Robot::setHeading(uint8_t heading) {
    if (heading > 3) {
        return;
    }

    _state.heading = heading;
}

void Robot::notifyCellReached(uint8_t x, uint8_t y) {
    if (!_navigation.gridMap().isValidPosition(x, y)) {
        char error[128];
        snprintf(error, sizeof(error), "Cell reached outside grid (%u,%u)", x, y);
        enterError(error);
        return;
    }

    _state.x = x;
    _state.y = y;
    Serial.printf("ROBOT CELL REACHED: (%u,%u)\n", x, y);
    syncPositionFromRoute();
}

void Robot::notifyMovementFailed(const char* message) {
    char error[128];
    snprintf(error, sizeof(error), "%s", message && message[0] != '\0' ? message : "Movement command failed");
    enterError(error);
}

void Robot::notifyObstacleDetected(uint8_t x, uint8_t y) {
    Serial.printf("OBSTACLE DETECTED: (%u,%u)\n", x, y);
    _obstacleX = x;
    _obstacleY = y;
    _hasPendingObstacle = true;
    _state.obstacleDetected = true;

    if (_state.autonomousState == AutonomousState::MOVING ||
        _state.autonomousState == AutonomousState::RETURNING_HOME) {
        transitionTo(AutonomousState::AVOIDING_OBSTACLE, _stateStartedAt);
    }
}

void Robot::clearObstacleDetection() {
    _hasPendingObstacle = false;
    _state.obstacleDetected = false;
}

void Robot::notifyFireDetected(bool detected) {
    _state.fireDetected = detected;
}

void Robot::notifyFireConfirmed(unsigned long currentMillis) {
    if (_state.autonomousState != AutonomousState::MOVING &&
        _state.autonomousState != AutonomousState::SEARCHING_FIRE) {
        return;
    }

    Serial.println("FIRE CONFIRMED: activating pump");
    _state.fireDetected = true;
    _state.routeReady = false;
    transitionTo(AutonomousState::EXTINGUISHING_FIRE, currentMillis);
}

void Robot::notifyFireSearchTimedOut() {
    if (_state.autonomousState != AutonomousState::SEARCHING_FIRE) {
        return;
    }

    Serial.println("FIRE SEARCH TIMEOUT: preparing reverse return");
    _state.returningHome = true;
    _state.fireDetected = false;
    _state.targetX = navigation::GridMap::Home.x;
    _state.targetY = navigation::GridMap::Home.y;
    _state.hasTarget = true;
    _state.routeReady = false;
    prepareReverseReturnRoute();
    transitionTo(AutonomousState::RETURNING_HOME, _stateStartedAt);
}

void Robot::notifyExtinguishingComplete() {
    _extinguishingComplete = true;
}

void Robot::resetError() {
    _state.error = false;
    _state.mode = RobotMode::Autonomous;
    setLastError("");
    transitionTo(AutonomousState::IDLE, _stateStartedAt);
}

const RobotState& Robot::state() const {
    return _state;
}

void Robot::setMode(RobotMode mode) {
    _state.mode = mode;

    if (mode == RobotMode::Error) {
        transitionTo(AutonomousState::ERROR, _stateStartedAt);
    }
}

AutonomousState Robot::autonomousState() const {
    return _state.autonomousState;
}

bool Robot::hasNextWaypoint() const {
    const auto& path = _navigation.currentPath();
    if (_state.returningHome) {
        return _state.routeReady &&
               !path.empty() &&
               !reachedHome() &&
               _pathIndex < path.size();
    }

    return _pathIndex < path.size();
}

navigation::Coordinate Robot::nextWaypoint() const {
    if (!hasNextWaypoint()) {
        return {_state.x, _state.y};
    }

    return _navigation.currentPath()[_pathIndex];
}

const std::vector<navigation::Coordinate>& Robot::currentPath() const {
    return _navigation.currentPath();
}

const char* Robot::lastError() const {
    return _state.lastError;
}

void Robot::transitionTo(AutonomousState nextState, unsigned long currentMillis) {
    if (_state.autonomousState != nextState) {
        Serial.printf("STATE -> %s\n", stateName(nextState));
    } else {
        Serial.printf("STATE -> %s (unchanged)\n", stateName(nextState));
    }

    _state.autonomousState = nextState;
    _stateStartedAt = currentMillis;
}

void Robot::handleReceivingTarget(unsigned long currentMillis) {
    if (!_state.hasTarget) {
        transitionTo(AutonomousState::IDLE, currentMillis);
        return;
    }

    transitionTo(AutonomousState::CALCULATING_ROUTE, currentMillis);
}

void Robot::handleCalculatingRoute(unsigned long currentMillis) {
    if (!_state.hasTarget) {
        transitionTo(AutonomousState::IDLE, currentMillis);
        return;
    }

    if (!calculateRouteTo(_state.targetX, _state.targetY)) {
        enterError(_state.lastError);
        return;
    }

    transitionTo(AutonomousState::MOVING, currentMillis);
}

void Robot::handleMoving(unsigned long currentMillis) {
    if (_state.obstacleDetected) {
        transitionTo(AutonomousState::AVOIDING_OBSTACLE, currentMillis);
        return;
    }

    if (reachedTarget()) {
        Serial.println("TARGET REACHED: starting fire scan");

        transitionTo(AutonomousState::SEARCHING_FIRE, currentMillis);
    }
}

void Robot::handleAvoidingObstacle(unsigned long currentMillis) {
    if (_hasPendingObstacle) {
        if (!_navigation.gridMap().setDynamicObstacle(_obstacleX, _obstacleY, true)) {
            char error[128];
            snprintf(error, sizeof(error), "Could not mark dynamic obstacle at (%u,%u)", _obstacleX, _obstacleY);
            enterError(error);
            return;
        }
    }

    clearObstacleDetection();
    _state.routeReady = false;
    _routeRecalculationPending = true;
    Serial.println("GRID AFTER RECALCULATE REQUEST");
    _navigation.gridMap().debugPrintGrid(_state.x, _state.y, _state.targetX, _state.targetY, _state.hasTarget);

    if (_state.returningHome) {
        if (!calculateRouteTo(navigation::GridMap::Home.x, navigation::GridMap::Home.y)) {
            enterError(_state.lastError);
            return;
        }

        // Novo BFS e avanco celula a celula; nao usar marcha a re no caminho replanejado.
        _state.returningHome = false;
        transitionTo(AutonomousState::RETURNING_HOME, currentMillis);
    } else {
        transitionTo(AutonomousState::CALCULATING_ROUTE, currentMillis);
    }
}

void Robot::handleSearchingFire(unsigned long currentMillis) {
    if (_state.fireDetected) {
        Serial.println("FIRE CONFIRMED: activating pump");

        transitionTo(AutonomousState::EXTINGUISHING_FIRE, currentMillis);
    }
}

void Robot::handleExtinguishingFire(unsigned long currentMillis) {
    if (!_extinguishingComplete) {
        return;
    }

    _state.returningHome = true;
    _state.fireDetected = false;
    _state.targetX = navigation::GridMap::Home.x;
    _state.targetY = navigation::GridMap::Home.y;
    _state.hasTarget = true;
    _state.routeReady = false;
    _extinguishingComplete = false;
    prepareReverseReturnRoute();
    transitionTo(AutonomousState::RETURNING_HOME, currentMillis);
}

void Robot::handleReturningHome(unsigned long currentMillis) {
    if (_state.obstacleDetected) {
        transitionTo(AutonomousState::AVOIDING_OBSTACLE, currentMillis);
        return;
    }

    if (reachedHome()) {
        _state.hasTarget = false;
        _state.returningHome = false;
        _state.routeReady = false;
        transitionTo(AutonomousState::IDLE, currentMillis);
        return;
    }

    if (!_state.routeReady) {
        if (calculateRouteTo(navigation::GridMap::Home.x, navigation::GridMap::Home.y)) {
            _state.returningHome = false;
            return;
        }

        enterError(_state.lastError[0] != '\0' ? _state.lastError : "Return route is not ready");
        return;
    }

    if (!hasNextWaypoint() && !reachedHome()) {
        enterError("Return stalled: no valid waypoint from current position");
    }
}

void Robot::enterError(const char* message) {
    setLastError(message && message[0] != '\0' ? message : "Unknown robot error");
    _state.error = true;
    _state.mode = RobotMode::Error;
    _state.routeReady = false;
    Serial.print("ERROR DETAIL: ");
    Serial.println(_state.lastError);
    transitionTo(AutonomousState::ERROR, _stateStartedAt);
}

bool Robot::calculateRouteTo(uint8_t x, uint8_t y) {
    Serial.printf("ROUTE REQUEST: from (%u,%u) to (%u,%u)\n", _state.x, _state.y, x, y);

    if (!_navigation.gridMap().isValidPosition(_state.x, _state.y)) {
        char error[128];
        snprintf(error, sizeof(error), "Route failed: current position outside grid (%u,%u)", _state.x, _state.y);
        setLastError(error);
        Serial.println(_state.lastError);
        return false;
    }

    if (!_navigation.gridMap().isCellFree(_state.x, _state.y)) {
        char error[128];
        snprintf(error, sizeof(error), "Route failed: current position blocked (%u,%u)", _state.x, _state.y);
        setLastError(error);
        Serial.println(_state.lastError);
        return false;
    }

    if (!_navigation.gridMap().isValidPosition(x, y)) {
        char error[128];
        snprintf(error, sizeof(error), "Route failed: target outside grid (%u,%u)", x, y);
        setLastError(error);
        Serial.println(_state.lastError);
        return false;
    }

    if (!_navigation.gridMap().isCellFree(x, y) && !(_state.x == x && _state.y == y)) {
        char error[128];
        snprintf(error, sizeof(error), "Route failed: target blocked (%u,%u)", x, y);
        setLastError(error);
        Serial.println(_state.lastError);
        return false;
    }

    if (!_navigation.requestPath(_state.x, _state.y, x, y)) {
        _state.routeReady = false;
        _pathIndex = 0;
        setLastError(_navigation.lastError());
        return false;
    }

    if (_navigation.currentPath().empty()) {
        _state.routeReady = false;
        _pathIndex = 0;
        setLastError("Route failed: BFS returned empty path");
        Serial.println(_state.lastError);
        return false;
    }

    _state.routeReady = true;
    _pathIndex = _navigation.currentPath().size() > 1 ? 1 : 0;
    setLastError("");
    Serial.printf("ROUTE READY: pathSize=%u nextIndex=%u\n",
                  static_cast<unsigned>(_navigation.currentPath().size()),
                  _pathIndex);
    Serial.println(_routeRecalculationPending ? "GRID AFTER ROUTE RECALCULATED" : "GRID AFTER ROUTE CALCULATED");
    _navigation.gridMap().debugPrintGrid(_state.x, _state.y, x, y, true);
    _routeRecalculationPending = false;
    return true;
}

bool Robot::reachedTarget() const {
    return _state.x == _state.targetX && _state.y == _state.targetY;
}

bool Robot::reachedHome() const {
    return _state.x == navigation::GridMap::Home.x && _state.y == navigation::GridMap::Home.y;
}

void Robot::prepareReverseReturnRoute() {
    const auto& path = _navigation.currentPath();

    if (reachedHome()) {
        _state.routeReady = false;
        _pathIndex = 0;
        return;
    }

    if (path.size() < 2) {
        _state.routeReady = false;
        _pathIndex = 0;
        setLastError("Reverse return failed: current route is too short");
        Serial.println(_state.lastError);
        return;
    }

    uint8_t currentIndex = 0;
    bool foundCurrentPosition = false;

    for (uint8_t i = 0; i < path.size(); ++i) {
        if (path[i].x == _state.x && path[i].y == _state.y) {
            currentIndex = i;
            foundCurrentPosition = true;
            break;
        }
    }

    if (!foundCurrentPosition || currentIndex == 0) {
        _state.routeReady = false;
        _pathIndex = 0;
        setLastError("Reverse return failed: current position is not on the outgoing route");
        Serial.println(_state.lastError);
        return;
    }

    _state.routeReady = true;
    _pathIndex = currentIndex - 1;
    setLastError("");
    Serial.printf("REVERSE ROUTE READY: pathSize=%u nextIndex=%u\n",
                  static_cast<unsigned>(path.size()),
                  _pathIndex);
}

void Robot::syncPositionFromRoute() {
    const auto& path = _navigation.currentPath();

    if (_state.returningHome) {
        if (_pathIndex < path.size() &&
            path[_pathIndex].x == _state.x &&
            path[_pathIndex].y == _state.y &&
            _pathIndex > 0) {
            --_pathIndex;
        }

        return;
    }

    while (_pathIndex < path.size() &&
           path[_pathIndex].x == _state.x &&
           path[_pathIndex].y == _state.y) {
        ++_pathIndex;
    }
}

void Robot::setLastError(const char* message) {
    if (!message) {
        _state.lastError[0] = '\0';
        return;
    }

    snprintf(_state.lastError, sizeof(_state.lastError), "%s", message);
}

const char* Robot::stateName(AutonomousState state) {
    switch (state) {
        case AutonomousState::IDLE:
            return "IDLE";
        case AutonomousState::RECEIVING_TARGET:
            return "RECEIVING_TARGET";
        case AutonomousState::CALCULATING_ROUTE:
            return "CALCULATING_ROUTE";
        case AutonomousState::MOVING:
            return "MOVING";
        case AutonomousState::AVOIDING_OBSTACLE:
            return "AVOIDING_OBSTACLE";
        case AutonomousState::SEARCHING_FIRE:
            return "SEARCHING_FIRE";
        case AutonomousState::EXTINGUISHING_FIRE:
            return "EXTINGUISHING_FIRE";
        case AutonomousState::RETURNING_HOME:
            return "RETURNING_HOME";
        case AutonomousState::ERROR:
            return "ERROR";
    }

    return "ERROR";
}

} // namespace robot
