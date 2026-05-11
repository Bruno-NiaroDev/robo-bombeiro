#pragma once

#include <stdint.h>
#include "drivers/MotorDriver.h"

namespace movement {

enum class Orientation {
    NORTH,
    SOUTH,
    EAST,
    WEST
};

struct GridPosition {
    int16_t x;
    int16_t y;
};

class MovementController {
public:
    explicit MovementController(drivers::MotorDriver& motorDriver);

    void begin();
    void update(unsigned long currentMillis);

    bool moveForwardOneCell();
    bool moveToAdjacentCell(int16_t targetX, int16_t targetY);
    bool turnLeft90();
    bool turnRight90();

    void stop();

    bool isBusy() const;
    bool isMoving() const;
    bool isPaused() const;

    GridPosition position() const;
    Orientation orientation() const;

    void setPosition(int16_t x, int16_t y);
    void setOrientation(Orientation orientation);

    void setMoveDuration(unsigned long durationMillis);
    void setTurnDuration(unsigned long durationMillis);
    void setPauseDuration(unsigned long durationMillis);
    void setMoveSpeed(uint8_t speed);
    void setTurnSpeed(uint8_t speed);

private:
    enum class State {
        Idle,
        MovingForward,
        TurningLeft,
        TurningRight,
        Pausing
    };

    enum class PendingAction {
        None,
        TurnLeftThenMove,
        MoveForward
    };

    bool startForwardMove();
    bool startLeftTurn();
    bool startRightTurn();
    void startPause(PendingAction nextAction);
    void finishForwardMove();
    void finishLeftTurn();
    void finishRightTurn();
    void advancePosition();
    bool elapsed(unsigned long currentMillis, unsigned long durationMillis);
    Orientation orientationForDelta(int16_t deltaX, int16_t deltaY) const;
    uint8_t rightTurnsTo(Orientation targetOrientation) const;
    static Orientation turnLeft(Orientation orientation);
    static Orientation turnRight(Orientation orientation);
    static const char* orientationName(Orientation orientation);
    static const char* stateName(State state);
    void logState(State nextState);

    drivers::MotorDriver& _motorDriver;
    GridPosition _position = {0, 0};
    Orientation _orientation = Orientation::SOUTH;
    State _state = State::Idle;
    PendingAction _pendingAction = PendingAction::None;
    unsigned long _stateStartedAt = 0;
    unsigned long _moveDurationMillis = 900;
    unsigned long _turnDurationMillis = 450;
    unsigned long _pauseDurationMillis = 150;
    uint8_t _moveSpeed = 180;
    uint8_t _turnSpeed = 160;
};

} // namespace movement
