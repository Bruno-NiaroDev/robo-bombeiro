#pragma once

#include "actuators/PumpController.h"
#include "drivers/MotorDriver.h"
#include "movement/MovementController.h"
#include "network/NetworkManager.h"
#include "robot/Robot.h"
#include "sensors/FireScanner.h"
#include "sensors/UltrasonicSensor.h"

namespace robot {

class AutonomousRobotSystem {
public:
    AutonomousRobotSystem();

    void begin();
    void update(unsigned long currentMillis);

    const RobotState& state() const;

private:
    void updateModules(unsigned long currentMillis);
    void processNetworkCommands(unsigned long currentMillis);
    void processMovementEvents();
    void processSensorEvents(unsigned long currentMillis);
    void processPumpEvents();
    void executeStateActions(unsigned long currentMillis);
    void commandNextCell();
    void startExtinguishing(unsigned long currentMillis);
    void startReturningHome();
    void sendTelemetry(unsigned long currentMillis);
    const char* stateName() const;

    static const unsigned long ExtinguishingDurationMillis = 3000;
    static const unsigned long ObstacleConfirmationMillis = 5000;
    static const unsigned long FireSearchDurationMillis = 10000;

    drivers::MotorDriver _motorDriver;
    movement::MovementController _movement;
    sensors::UltrasonicSensor _ultrasonic;
    sensors::FireScanner _fireScanner;
    actuators::PumpController _pump;
    network::NetworkManager _network;
    Robot _robot;

    navigation::Coordinate _activeWaypoint = {0, 0};
    navigation::Coordinate _pendingObstacle = {0, 0};
    bool _obstacleConfirmationActive = false;
    bool _movementCommandActive = false;
    bool _pumpCommandActive = false;
    bool _errorTelemetrySent = false;
    bool _fireSearchActive = false;
    unsigned long _obstacleDetectedAt = 0;
    unsigned long _fireSearchStartedAt = 0;
    unsigned long _lastTelemetryMillis = 0;
};

} // namespace robot
