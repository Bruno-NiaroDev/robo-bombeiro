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
    void processNetworkCommands();
    void processMovementEvents();
    void processSensorEvents();
    void processPumpEvents();
    void executeStateActions(unsigned long currentMillis);
    void commandNextCell();
    void startExtinguishing(unsigned long currentMillis);
    void sendTelemetry(unsigned long currentMillis);
    const char* stateName() const;

    static const unsigned long ExtinguishingDurationMillis = 3000;

    drivers::MotorDriver _motorDriver;
    movement::MovementController _movement;
    sensors::UltrasonicSensor _ultrasonic;
    sensors::FireScanner _fireScanner;
    actuators::PumpController _pump;
    network::NetworkManager _network;
    Robot _robot;

    navigation::Coordinate _activeWaypoint = {0, 0};
    bool _movementCommandActive = false;
    bool _pumpCommandActive = false;
    bool _errorTelemetrySent = false;
    unsigned long _lastTelemetryMillis = 0;
};

} // namespace robot
