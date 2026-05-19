#include "robot/AutonomousRobotSystem.h"
#include "config/ConfigManager.h"
#include <Arduino.h>

namespace robot {

AutonomousRobotSystem::AutonomousRobotSystem()
    : _movement(_motorDriver) {}

void AutonomousRobotSystem::begin() {
    config::ConfigManager::instance().begin();

    _motorDriver.begin();
    _movement.begin();
    const config::NavigationConfig& navigationConfig = config::ConfigManager::instance().navigationConfig();
    _movement.setMoveDuration(navigationConfig.cellTravelDurationMs);
    _movement.setTurnDuration(navigationConfig.turn90DurationMs);
    _movement.setPauseDuration(navigationConfig.movementPauseDurationMs);
    _movement.setMoveSpeed(navigationConfig.moveSpeed);
    _movement.setTurnSpeed(navigationConfig.turnSpeed);
    _ultrasonic.begin();
    _fireScanner.begin();
    _pump.begin();
    _network.begin();
    _robot.begin();

    _movement.setPosition(navigation::GridMap::Home.x, navigation::GridMap::Home.y);
    _robot.setPosition(navigation::GridMap::Home.x, navigation::GridMap::Home.y);
    _activeWaypoint = {navigation::GridMap::Home.x, navigation::GridMap::Home.y};
    _pendingObstacle = _activeWaypoint;
    _obstacleConfirmationActive = false;
    _movementCommandActive = false;
    _pumpCommandActive = false;
    _errorTelemetrySent = false;
    _fireSearchActive = false;
    _lastTelemetryMillis = 0;
}

void AutonomousRobotSystem::update(unsigned long currentMillis) {
    updateModules(currentMillis);
    processNetworkCommands();
    processMovementEvents();
    processSensorEvents(currentMillis);
    processPumpEvents();

    _robot.update(currentMillis);
    executeStateActions(currentMillis);
    sendTelemetry(currentMillis);
}

const RobotState& AutonomousRobotSystem::state() const {
    return _robot.state();
}

void AutonomousRobotSystem::updateModules(unsigned long currentMillis) {
    config::ConfigManager::instance().update(currentMillis);
    _network.update(currentMillis);
    _movement.update(currentMillis);
    _ultrasonic.update(currentMillis);
    _fireScanner.update(currentMillis);
    _pump.update(currentMillis);
}

void AutonomousRobotSystem::processNetworkCommands() {
    if (!_network.hasTargetPosition()) {
        return;
    }

    network::TargetPositionMessage target = _network.consumeTargetPosition();

    if (target.available) {
        Serial.printf("NETWORK TARGET CONSUMED: (%u,%u)\n", target.x, target.y);
        _movement.stop();
        _movement.setPosition(_robot.state().x, _robot.state().y);
        _robot.setTarget(target.x, target.y);
        _movementCommandActive = false;
        _obstacleConfirmationActive = false;
        _pumpCommandActive = false;
        _errorTelemetrySent = false;
        _fireSearchActive = false;
        _pump.turnOff();
        _fireScanner.enableDetection(false);
        _fireScanner.enableSweep(false);
        _fireScanner.center();
    }
}

void AutonomousRobotSystem::processMovementEvents() {
    if (!_movementCommandActive) {
        return;
    }

    if (!_movement.isBusy()) {
        movement::GridPosition position = _movement.position();
        _robot.notifyCellReached(static_cast<uint8_t>(position.x), static_cast<uint8_t>(position.y));
        _movementCommandActive = false;
    }
}

void AutonomousRobotSystem::processSensorEvents(unsigned long currentMillis) {
    bool navigationState = _robot.autonomousState() == AutonomousState::MOVING ||
                           _robot.autonomousState() == AutonomousState::RETURNING_HOME;
    bool searchingFire = _robot.autonomousState() == AutonomousState::SEARCHING_FIRE;
    bool shouldDetectFire = true;

    if (shouldDetectFire) {
        _fireScanner.enableDetection(true);
    } else {
        _fireScanner.enableDetection(false);
    }

    if (searchingFire && !_fireScanner.sweepEnabled()) {
        _fireScanner.enableSweep(true);
        _fireSearchStartedAt = currentMillis;
        _fireSearchActive = true;
    } else if (!searchingFire && _fireScanner.sweepEnabled()) {
        _fireScanner.enableSweep(false);
        _fireSearchActive = false;
    }

    if (_ultrasonic.hasNewData()) {
        if (_movementCommandActive && navigationState && _ultrasonic.obstacleDetected()) {
            _movement.stop();
            _movementCommandActive = false;
            _pendingObstacle = _activeWaypoint;
            _obstacleDetectedAt = currentMillis;
            _obstacleConfirmationActive = true;
            Serial.printf("OBSTACLE WAIT: candidate=(%u,%u) confirmationMs=%lu\n",
                          _pendingObstacle.x,
                          _pendingObstacle.y,
                          ObstacleConfirmationMillis);
        } else if (_obstacleConfirmationActive && navigationState) {
            if (!_ultrasonic.obstacleDetected()) {
                Serial.printf("OBSTACLE CLEARED: candidate=(%u,%u)\n",
                              _pendingObstacle.x,
                              _pendingObstacle.y);
                _obstacleConfirmationActive = false;
            } else if (currentMillis - _obstacleDetectedAt >= ObstacleConfirmationMillis) {
                Serial.printf("OBSTACLE CONFIRMED: candidate=(%u,%u)\n",
                              _pendingObstacle.x,
                              _pendingObstacle.y);
                _robot.notifyObstacleDetected(_pendingObstacle.x, _pendingObstacle.y);
                _obstacleConfirmationActive = false;
            }
        }

        _ultrasonic.clearNewData();
    }

    if (shouldDetectFire) {
        bool fireConfirmed = _fireScanner.fireDetected();
        _robot.notifyFireDetected(fireConfirmed);

        if (fireConfirmed) {
            _fireScanner.enableSweep(false);
            _fireSearchActive = false;
            _robot.notifyFireConfirmed(currentMillis);
        } else if (_fireSearchActive &&
                   currentMillis - _fireSearchStartedAt >= FireSearchDurationMillis) {
            Serial.println("FIRE SEARCH TIMEOUT: returning home");
            _fireScanner.enableSweep(false);
            _fireScanner.center();
            _fireSearchActive = false;
            startReturningHome();
        }
    }

    if (_fireScanner.hasNewData()) {
        _fireScanner.clearNewData();
    }
}

void AutonomousRobotSystem::processPumpEvents() {
    if (_pumpCommandActive && !_pump.isOn()) {
        _pumpCommandActive = false;
        _fireScanner.center();
        _robot.notifyExtinguishingComplete();
    }
}

void AutonomousRobotSystem::executeStateActions(unsigned long currentMillis) {
    switch (_robot.autonomousState()) {
        case AutonomousState::MOVING:
        case AutonomousState::RETURNING_HOME:
            commandNextCell();
            break;

        case AutonomousState::EXTINGUISHING_FIRE:
            startExtinguishing(currentMillis);
            break;

        case AutonomousState::IDLE:
        case AutonomousState::RECEIVING_TARGET:
        case AutonomousState::CALCULATING_ROUTE:
        case AutonomousState::AVOIDING_OBSTACLE:
        case AutonomousState::SEARCHING_FIRE:
        case AutonomousState::ERROR:
            break;
    }
}

void AutonomousRobotSystem::commandNextCell() {
    if (_movementCommandActive ||
        _obstacleConfirmationActive ||
        _movement.isBusy() ||
        !_robot.state().routeReady ||
        !_robot.hasNextWaypoint()) {
        return;
    }

    navigation::Coordinate waypoint = _robot.nextWaypoint();
    const RobotState& robotState = _robot.state();

    Serial.printf("NEXT WAYPOINT: current=(%u,%u) waypoint=(%u,%u) pathSize=%u\n",
                  robotState.x,
                  robotState.y,
                  waypoint.x,
                  waypoint.y,
                  static_cast<unsigned>(_robot.currentPath().size()));

    bool commandAccepted = _robot.autonomousState() == AutonomousState::RETURNING_HOME
        ? _movement.moveBackwardToAdjacentCell(waypoint.x, waypoint.y)
        : _movement.moveToAdjacentCell(waypoint.x, waypoint.y);

    if (commandAccepted) {
        _activeWaypoint = waypoint;
        _movementCommandActive = true;
    } else {
        Serial.printf("MOVEMENT COMMAND FAILED: waypoint=(%u,%u) robot=(%u,%u)\n",
                      waypoint.x,
                      waypoint.y,
                      robotState.x,
                      robotState.y);
    }
}

void AutonomousRobotSystem::startExtinguishing(unsigned long currentMillis) {
    if (_pumpCommandActive || _pump.isOn()) {
        return;
    }

    _fireScanner.enableSweep(false);
    _fireScanner.enableDetection(false);
    _fireSearchActive = false;
    _pumpCommandActive = _pump.runFor(ExtinguishingDurationMillis, currentMillis);
}

void AutonomousRobotSystem::startReturningHome() {
    _movement.stop();
    _movement.setPosition(_robot.state().x, _robot.state().y);
    _robot.notifyFireSearchTimedOut();
    _movementCommandActive = false;
    _obstacleConfirmationActive = false;
}

void AutonomousRobotSystem::sendTelemetry(unsigned long currentMillis) {
    const config::NetworkConfig& networkConfig = config::ConfigManager::instance().networkConfig();
    bool errorState = _robot.autonomousState() == AutonomousState::ERROR;
    bool shouldSendErrorNow = errorState && !_errorTelemetrySent;

    if (!errorState) {
        _errorTelemetrySent = false;
    }

    if (!shouldSendErrorNow &&
        (networkConfig.telemetryIntervalMs == 0 ||
         currentMillis - _lastTelemetryMillis < networkConfig.telemetryIntervalMs)) {
        return;
    }

    _lastTelemetryMillis = currentMillis;

    const RobotState& robotState = _robot.state();
    network::TelemetryPayload telemetry;
    telemetry.x = robotState.x;
    telemetry.y = robotState.y;
    telemetry.state = stateName();
    telemetry.pathSize = static_cast<uint8_t>(_robot.currentPath().size());
    telemetry.targetX = robotState.targetX;
    telemetry.targetY = robotState.targetY;
    telemetry.obstacleDetected = robotState.obstacleDetected;
    telemetry.fireDetected = robotState.fireDetected;
    telemetry.pumpOn = _pump.isOn();
    telemetry.lastError = _robot.lastError();

    bool sent = _network.sendTelemetry(telemetry);

    if (errorState) {
        _errorTelemetrySent = sent;
        Serial.printf("ERROR TELEMETRY %s: lastError=%s\n",
                      sent ? "SENT" : "PENDING",
                      telemetry.lastError);
    }
}

const char* AutonomousRobotSystem::stateName() const {
    switch (_robot.autonomousState()) {
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
