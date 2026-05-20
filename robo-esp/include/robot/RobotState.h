#pragma once

#include <stdint.h>

namespace robot {

enum class RobotMode {
    Idle,
    Autonomous,
    RemoteControl,
    Error
};

enum class AutonomousState {
    IDLE,
    RECEIVING_TARGET,
    CALCULATING_ROUTE,
    MOVING,
    AVOIDING_OBSTACLE,
    SEARCHING_FIRE,
    EXTINGUISHING_FIRE,
    RETURNING_HOME,
    ERROR
};

struct RobotState {
    RobotMode mode = RobotMode::Idle;
    AutonomousState autonomousState = AutonomousState::IDLE;
    uint8_t x = 0;
    uint8_t y = 0;
    uint8_t heading = 2; // 0=N 1=E 2=S 3=W (alinhado a MovementController, default SOUTH)
    uint8_t targetX = 0;
    uint8_t targetY = 0;
    bool hasTarget = false;
    bool routeReady = false;
    bool fireDetected = false;
    bool obstacleDetected = false;
    bool returningHome = false;
    bool error = false;
    char lastError[128] = "";
};

} // namespace robot
