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
    _ultrasonic.begin();
    _fireScanner.begin();
    _pump.begin();
    _network.begin();
    _robot.begin();

    _movement.setPosition(navigation::GridMap::Home.x, navigation::GridMap::Home.y);
    _robot.setPosition(navigation::GridMap::Home.x, navigation::GridMap::Home.y);
    _activeWaypoint = {navigation::GridMap::Home.x, navigation::GridMap::Home.y};
    _movementCommandActive = false;
    _pumpCommandActive = false;
    _errorTelemetrySent = false;
    _lastTelemetryMillis = 0;
}

void AutonomousRobotSystem::update(unsigned long currentMillis) {
    updateModules(currentMillis);
    processNetworkCommands();
    processMovementEvents();
    processSensorEvents();
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
        _pumpCommandActive = false;
        _errorTelemetrySent = false;
        _pump.turnOff();
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

void AutonomousRobotSystem::processSensorEvents() {
    bool navigationState = _robot.autonomousState() == AutonomousState::MOVING ||
                           _robot.autonomousState() == AutonomousState::RETURNING_HOME;

    if (_ultrasonic.hasNewData()) {
        if (_movementCommandActive && navigationState && _ultrasonic.obstacleDetected()) {
            _movement.stop();
            _robot.notifyObstacleDetected(_activeWaypoint.x, _activeWaypoint.y);
            _movementCommandActive = false;
        }

        _ultrasonic.clearNewData();
    }

    if (_robot.autonomousState() == AutonomousState::SEARCHING_FIRE) {
        _robot.notifyFireDetected(_fireScanner.fireDetected());
    }

    if (_fireScanner.hasNewData()) {
        _fireScanner.clearNewData();
    }
}

void AutonomousRobotSystem::processPumpEvents() {
    if (_pumpCommandActive && !_pump.isOn()) {
        _pumpCommandActive = false;
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

    if (_movement.moveToAdjacentCell(waypoint.x, waypoint.y)) {
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

    _pumpCommandActive = _pump.runFor(ExtinguishingDurationMillis, currentMillis);
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
