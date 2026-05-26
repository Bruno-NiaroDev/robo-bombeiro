#include "robot/AutonomousRobotSystem.h"
#include "config/ConfigManager.h"
#include "Pins.h"
#include <Arduino.h>

namespace robot {

namespace {

uint8_t headingFromOrientation(movement::Orientation orientation) {
    switch (orientation) {
        case movement::Orientation::NORTH:
            return 0;
        case movement::Orientation::EAST:
            return 1;
        case movement::Orientation::SOUTH:
            return 2;
        case movement::Orientation::WEST:
            return 3;
    }

    return 2;
}

movement::Orientation orientationFromHeading(uint8_t heading) {
    switch (heading) {
        case 0:
            return movement::Orientation::NORTH;
        case 1:
            return movement::Orientation::EAST;
        case 3:
            return movement::Orientation::WEST;
        default:
            return movement::Orientation::SOUTH;
    }
}

void syncMovementFromRobot(movement::MovementController& movement, const RobotState& robotState) {
    movement.setPosition(static_cast<int16_t>(robotState.x), static_cast<int16_t>(robotState.y));
    movement.setOrientation(orientationFromHeading(robotState.heading));
}

} // namespace

AutonomousRobotSystem::AutonomousRobotSystem()
    : _movement(_motorDriver) {}

void AutonomousRobotSystem::begin() {
    config::ConfigManager::instance().begin();

    _motorDriver.begin();
    if (_imu.begin(pins::sensors::imu::SDA, pins::sensors::imu::SCL)) {
        _movement.setImu(&_imu);
    } else {
        Serial.println("AVISO: IMU nao disponivel, navegacao por timer apenas");
    }
    _movement.begin();
    const config::NavigationConfig& navigationConfig = config::ConfigManager::instance().navigationConfig();
    _movement.setMoveDuration(navigationConfig.cellTravelDurationMs);
    _movement.setTurnDuration(navigationConfig.turn90DurationMs);
    _movement.setPauseDuration(navigationConfig.movementPauseDurationMs);
    _movement.setMoveSpeed(navigationConfig.moveSpeed);
    _movement.setTurnSpeed(navigationConfig.turnSpeed);
    _movement.setMinTurnSpeed(navigationConfig.minTurnSpeed);
    _movement.setTurnTolerance(navigationConfig.turnToleranceDeg);
    _movement.setTurnPid(navigationConfig.turnPidKp,
                         navigationConfig.turnPidKi,
                         navigationConfig.turnPidKd);
    _movement.setHeadingPid(navigationConfig.headingPidKp,
                            navigationConfig.headingPidKi,
                            navigationConfig.headingPidKd);
    _movement.setTurnLeftPositiveYaw(navigationConfig.turnLeftPositiveYaw);
    _imu.setYawInverted(navigationConfig.imuYawInverted);
    _imu.setFilterAlpha(navigationConfig.imuFilterAlpha);
    if (_imu.isReady()) {
        _imu.resetYaw();
        Serial.println("IMU: yaw zerado na orientacao inicial");
    }

    // Aplica a orientação física do robô ao ligar
    // 0=NORTH 1=EAST 2=SOUTH 3=WEST — ajuste em NavigationConfig::initialOrientationValue
    movement::Orientation initOrientation;
    switch (navigationConfig.initialOrientationValue) {
        case 0: initOrientation = movement::Orientation::NORTH; break;
        case 1: initOrientation = movement::Orientation::EAST;  break;
        case 3: initOrientation = movement::Orientation::WEST;  break;
        default: initOrientation = movement::Orientation::SOUTH; break;
    }
    _movement.setOrientation(initOrientation);
    Serial.printf("ORIENTACAO INICIAL: %s (value=%u)\n",
                  navigationConfig.initialOrientationValue == 0 ? "NORTH" :
                  navigationConfig.initialOrientationValue == 1 ? "EAST"  :
                  navigationConfig.initialOrientationValue == 3 ? "WEST"  : "SOUTH",
                  navigationConfig.initialOrientationValue);
    _ultrasonic.begin();
    _fireScanner.begin();
    _pump.begin();
    _pump.setMaxRunTime(ExtinguishingDurationMillis + 2000); // margem de 2s acima da duração configurada
    _network.begin();
    _robot.begin();

    const config::RobotConfig& robotConfig = config::ConfigManager::instance().robotConfig();
    if (robotConfig.startX != navigation::GridMap::Home.x ||
        robotConfig.startY != navigation::GridMap::Home.y) {
        Serial.printf("CONFIG WARNING: start=(%u,%u) difere de Home=(%u,%u); usando Home\n",
                      robotConfig.startX,
                      robotConfig.startY,
                      navigation::GridMap::Home.x,
                      navigation::GridMap::Home.y);
    }

    _movement.setPosition(navigation::GridMap::Home.x, navigation::GridMap::Home.y);
    _robot.setPosition(navigation::GridMap::Home.x, navigation::GridMap::Home.y);
    _robot.setHeading(headingFromOrientation(_movement.orientation()));
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
    processNetworkCommands(currentMillis);
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
    _imu.update();
    _movement.update(currentMillis);
    _ultrasonic.update(currentMillis);
    _fireScanner.update(currentMillis);
    _pump.update(currentMillis);
}

void AutonomousRobotSystem::processNetworkCommands(unsigned long currentMillis) {
    if (!_network.hasTargetPosition()) {
        return;
    }

    network::TargetPositionMessage target = _network.consumeTargetPosition();

    if (target.available) {
        Serial.printf("NETWORK TARGET CONSUMED: (%u,%u)\n", target.x, target.y);
        _movement.stop();
        syncMovementFromRobot(_movement, _robot.state());
        _robot.setTarget(target.x, target.y, currentMillis);
        _movementCommandActive = false;
        _obstacleConfirmationActive = false;
        _pumpCommandActive = false;
        _errorTelemetrySent = false;
        _fireSearchActive = false;
        _pump.turnOff();
        _fireScanner.enableDetection(false, currentMillis);
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
        if (position.x != static_cast<int16_t>(_activeWaypoint.x) ||
            position.y != static_cast<int16_t>(_activeWaypoint.y)) {
            Serial.printf("MOVEMENT MISMATCH: reached=(%d,%d) waypoint=(%u,%u)\n",
                          position.x,
                          position.y,
                          _activeWaypoint.x,
                          _activeWaypoint.y);
        }
        _robot.notifyCellReached(static_cast<uint8_t>(position.x), static_cast<uint8_t>(position.y));
        _robot.setHeading(headingFromOrientation(_movement.orientation()));
        _movementCommandActive = false;
    }
}

void AutonomousRobotSystem::processSensorEvents(unsigned long currentMillis) {
    bool navigationState = _robot.autonomousState() == AutonomousState::MOVING ||
                           _robot.autonomousState() == AutonomousState::RETURNING_HOME;
    bool searchingFire   = _robot.autonomousState() == AutonomousState::SEARCHING_FIRE;
    bool extinguishing   = _robot.autonomousState() == AutonomousState::EXTINGUISHING_FIRE;
    bool shouldDetectFire = searchingFire; // detecção apenas na busca ativa, nunca durante navegação

    _fireScanner.enableDetection(shouldDetectFire, currentMillis);

    if (searchingFire && !_fireScanner.sweepEnabled()) {
        // Inicia varredura ao entrar no modo busca
        _fireScanner.enableSweep(true);
        _fireSearchStartedAt = currentMillis;
        _fireSearchActive = true;
    } else if (!searchingFire && !extinguishing && _fireScanner.sweepEnabled()) {
        // Desliga varredura ao sair da busca — mas mantém ativa durante extinção
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
            _movement.stop();
            _movementCommandActive = false;
            _obstacleConfirmationActive = false;
            _fireSearchActive = false;

            if (!_pumpCommandActive && !_pump.isOn()) {
                _fireScanner.enableSweep(false);                    // para servo H apontado para o fogo
                _fireScanner.enableDetection(false, currentMillis); // evita re-disparo
                _pumpCommandActive = _pump.runFor(ExtinguishingDurationMillis, currentMillis);
                Serial.printf("PUMP ON: fogo em %u graus, bomba por %lums\n",
                              _fireScanner.fireAngle(), ExtinguishingDurationMillis);
            }

            _robot.notifyFireConfirmed(currentMillis);
        } else if (_fireSearchActive &&
                   currentMillis - _fireSearchStartedAt >= FireSearchDurationMillis) {
            Serial.println("FIRE SEARCH TIMEOUT: returning home");
            _fireScanner.enableSweep(false);
            _fireScanner.center();
            _fireSearchActive = false;
            startReturningHome(currentMillis);
        }
    }

    if (_fireScanner.hasNewData()) {
        _fireScanner.clearNewData();
    }
}

void AutonomousRobotSystem::processPumpEvents() {
    if (_pumpCommandActive && !_pump.isOn()) {
        _pumpCommandActive = false;

        // Extinção concluída: para varredura e recolhe servos para posição de repouso
        _fireScanner.enableSweep(false);
        _fireScanner.center(); // horizontal → 0°, vertical → 0°
        Serial.println("EXTINGUISHING DONE: servos recolhidos, retornando home");

        _robot.notifyExtinguishingComplete(); // → RETURNING_HOME
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

    const RobotState& robotState = _robot.state();
    syncMovementFromRobot(_movement, robotState);

    navigation::Coordinate waypoint = _robot.nextWaypoint();

    Serial.printf("NEXT WAYPOINT: robot=(%u,%u) waypoint=(%u,%u) pathSize=%u heading=%u\n",
                  robotState.x,
                  robotState.y,
                  waypoint.x,
                  waypoint.y,
                  static_cast<unsigned>(_robot.currentPath().size()),
                  robotState.heading);

    const bool returningHome = _robot.autonomousState() == AutonomousState::RETURNING_HOME;
    bool commandAccepted = returningHome
        ? _movement.moveBackwardToAdjacentCell(waypoint.x, waypoint.y)
        : _movement.moveToAdjacentCell(waypoint.x, waypoint.y);

    if (commandAccepted) {
        _activeWaypoint = waypoint;
        _movementCommandActive = true;
    } else {
        char error[128];
        snprintf(error,
                 sizeof(error),
                 "Movement failed: robot=(%u,%u) waypoint=(%u,%u) returning=%s",
                 robotState.x,
                 robotState.y,
                 waypoint.x,
                 waypoint.y,
                 returningHome ? "true" : "false");
        Serial.println(error);
        _movement.stop();
        _movementCommandActive = false;
        _robot.notifyMovementFailed(error);
    }
}

void AutonomousRobotSystem::startExtinguishing(unsigned long currentMillis) {
    if (_pumpCommandActive || _pump.isOn()) {
        return;
    }

    // Fallback: bomba não foi ativada em processSensorEvents (não deve ocorrer normalmente).
    // Servo H já parado pela detecção; apenas garante que a bomba será ligada.
    _fireScanner.enableSweep(false);
    _fireScanner.enableDetection(false, currentMillis);
    _fireSearchActive = false;
    _pumpCommandActive = _pump.runFor(ExtinguishingDurationMillis, currentMillis);
    Serial.printf("EXTINGUISHING (fallback): bomba ligada por %lums\n",
                  ExtinguishingDurationMillis);
}

void AutonomousRobotSystem::startReturningHome(unsigned long currentMillis) {
    _movement.stop();
    syncMovementFromRobot(_movement, _robot.state());
    _robot.notifyFireSearchTimedOut(currentMillis);
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
    telemetry.heading = robotState.heading;
    telemetry.imuReady = _imu.isReady();
    telemetry.imuYaw = _imu.isReady() ? _imu.yaw() : 0.0f;
    telemetry.state = stateName();
    telemetry.pathSize = static_cast<uint8_t>(_robot.currentPath().size());
    telemetry.targetX = robotState.targetX;
    telemetry.targetY = robotState.targetY;
    telemetry.obstacleDetected = robotState.obstacleDetected;
    telemetry.fireDetected = robotState.fireDetected;
    telemetry.pumpOn = _pump.isOn();
    telemetry.flameRaw = _fireScanner.rawValue();
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
