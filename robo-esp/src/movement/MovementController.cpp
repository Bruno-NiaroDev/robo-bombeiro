#include "movement/MovementController.h"
#include "navigation/GridMap.h"
#include <Arduino.h>

namespace movement {

MovementController::MovementController(drivers::MotorDriver& motorDriver)
    : _motorDriver(motorDriver) {}

void MovementController::begin() {
    _position = {0, 0};
    _orientation = Orientation::SOUTH;
    _state = State::Idle;
    _pendingAction = PendingAction::None;
    _stateStartedAt = 0;
    _motorDriver.stop();
}

void MovementController::update(unsigned long currentMillis) {
    switch (_state) {
        case State::Idle:
            break;
        case State::MovingForward:
            if (elapsed(currentMillis, _moveDurationMillis)) {
                finishForwardMove();
                startPause(PendingAction::None);
            }
            break;
        case State::TurningLeft:
            if (elapsed(currentMillis, _turnDurationMillis)) {
                finishLeftTurn();
                startPause(_pendingAction);
            }
            break;
        case State::TurningRight:
            if (elapsed(currentMillis, _turnDurationMillis)) {
                finishRightTurn();
                startPause(_pendingAction);
            }
            break;
        case State::Pausing:
            if (elapsed(currentMillis, _pauseDurationMillis)) {
                PendingAction nextAction = _pendingAction;
                _pendingAction = PendingAction::None;
                logState(State::Idle);

                switch (nextAction) {
                    case PendingAction::None:
                        break;
                    case PendingAction::TurnLeftThenMove:
                        _pendingAction = PendingAction::MoveForward;
                        startLeftTurn();
                        break;
                    case PendingAction::MoveForward:
                        startForwardMove();
                        break;
                }
            }
            break;
    }

    _motorDriver.update(currentMillis);
}

bool MovementController::moveForwardOneCell() {
    if (_state != State::Idle) {
        Serial.printf("MOVEMENT REJECTED: controller busy state=%s\n", stateName(_state));
        return false;
    }

    Serial.printf("MOVEMENT COMMAND: forward one cell from (%d,%d) orientation=%s\n",
                  _position.x,
                  _position.y,
                  orientationName(_orientation));
    return startForwardMove();
}

bool MovementController::moveToAdjacentCell(int16_t targetX, int16_t targetY) {
    if (_state != State::Idle) {
        Serial.printf("MOVEMENT REJECTED: controller busy state=%s target=(%d,%d)\n",
                      stateName(_state),
                      targetX,
                      targetY);
        return false;
    }

    Serial.printf("MOVEMENT TARGET: current=(%d,%d) next=(%d,%d) orientation=%s\n",
                  _position.x,
                  _position.y,
                  targetX,
                  targetY,
                  orientationName(_orientation));

    if (targetX < 0 || targetY < 0 ||
        targetX >= navigation::GridMap::Width ||
        targetY >= navigation::GridMap::Height) {
        Serial.printf("MOVEMENT REJECTED: target outside grid (%d,%d)\n", targetX, targetY);
        return false;
    }

    int16_t deltaX = targetX - _position.x;
    int16_t deltaY = targetY - _position.y;

    if ((deltaX == 0 && deltaY == 0) || (deltaX != 0 && deltaY != 0)) {
        Serial.printf("MOVEMENT REJECTED: invalid adjacent delta dx=%d dy=%d diagonal=%s\n",
                      deltaX,
                      deltaY,
                      (deltaX != 0 && deltaY != 0) ? "true" : "false");
        return false;
    }

    if (deltaX < -1 || deltaX > 1 || deltaY < -1 || deltaY > 1) {
        Serial.printf("MOVEMENT REJECTED: target is not adjacent dx=%d dy=%d\n", deltaX, deltaY);
        return false;
    }

    Orientation targetOrientation = orientationForDelta(deltaX, deltaY);
    uint8_t turns = rightTurnsTo(targetOrientation);
    Serial.printf("MOVEMENT PLAN: targetOrientation=%s rightTurns=%u\n",
                  orientationName(targetOrientation),
                  turns);

    if (turns == 0) {
        return startForwardMove();
    }

    _pendingAction = PendingAction::MoveForward;

    if (turns == 1) {
        return startRightTurn();
    }

    if (turns == 2) {
        _pendingAction = PendingAction::TurnLeftThenMove;
        return startLeftTurn();
    }

    return startLeftTurn();
}

bool MovementController::turnLeft90() {
    if (_state != State::Idle) {
        Serial.printf("TURN LEFT REJECTED: controller busy state=%s\n", stateName(_state));
        return false;
    }

    return startLeftTurn();
}

bool MovementController::turnRight90() {
    if (_state != State::Idle) {
        Serial.printf("TURN RIGHT REJECTED: controller busy state=%s\n", stateName(_state));
        return false;
    }

    return startRightTurn();
}

void MovementController::stop() {
    _motorDriver.stop();
    logState(State::Idle);
    _pendingAction = PendingAction::None;
    Serial.printf("MOVEMENT STOP: position=(%d,%d) orientation=%s\n",
                  _position.x,
                  _position.y,
                  orientationName(_orientation));
}

bool MovementController::isBusy() const {
    return _state != State::Idle;
}

bool MovementController::isMoving() const {
    return _state == State::MovingForward ||
           _state == State::TurningLeft ||
           _state == State::TurningRight;
}

bool MovementController::isPaused() const {
    return _state == State::Pausing;
}

GridPosition MovementController::position() const {
    return _position;
}

Orientation MovementController::orientation() const {
    return _orientation;
}

void MovementController::setPosition(int16_t x, int16_t y) {
    if (_state == State::Idle) {
        _position = {x, y};
        Serial.printf("MOVEMENT POSITION SET: (%d,%d)\n", x, y);
    } else {
        Serial.printf("MOVEMENT POSITION SET REJECTED: busy state=%s\n", stateName(_state));
    }
}

void MovementController::setOrientation(Orientation orientation) {
    if (_state == State::Idle) {
        _orientation = orientation;
        Serial.printf("MOVEMENT ORIENTATION SET: %s\n", orientationName(_orientation));
    } else {
        Serial.printf("MOVEMENT ORIENTATION SET REJECTED: busy state=%s\n", stateName(_state));
    }
}

void MovementController::setMoveDuration(unsigned long durationMillis) {
    _moveDurationMillis = durationMillis;
}

void MovementController::setTurnDuration(unsigned long durationMillis) {
    _turnDurationMillis = durationMillis;
}

void MovementController::setPauseDuration(unsigned long durationMillis) {
    _pauseDurationMillis = durationMillis;
}

void MovementController::setMoveSpeed(uint8_t speed) {
    _moveSpeed = speed;
}

void MovementController::setTurnSpeed(uint8_t speed) {
    _turnSpeed = speed;
}

bool MovementController::startForwardMove() {
    _motorDriver.forward(_moveSpeed);
    logState(State::MovingForward);
    _stateStartedAt = 0;
    Serial.printf("MOVEMENT EXECUTED: forward speed=%u orientation=%s\n",
                  _moveSpeed,
                  orientationName(_orientation));
    return true;
}

bool MovementController::startLeftTurn() {
    _motorDriver.turnLeft(_turnSpeed);
    logState(State::TurningLeft);
    _stateStartedAt = 0;
    Serial.printf("ROTATION EXECUTED: left speed=%u from=%s\n", _turnSpeed, orientationName(_orientation));
    return true;
}

bool MovementController::startRightTurn() {
    _motorDriver.turnRight(_turnSpeed);
    logState(State::TurningRight);
    _stateStartedAt = 0;
    Serial.printf("ROTATION EXECUTED: right speed=%u from=%s\n", _turnSpeed, orientationName(_orientation));
    return true;
}

void MovementController::startPause(PendingAction nextAction) {
    _motorDriver.stop();
    _pendingAction = nextAction;
    logState(State::Pausing);
    _stateStartedAt = 0;
}

void MovementController::finishForwardMove() {
    _motorDriver.stop();
    advancePosition();
    Serial.printf("MOVEMENT FINISHED: position=(%d,%d) orientation=%s\n",
                  _position.x,
                  _position.y,
                  orientationName(_orientation));
}

void MovementController::finishLeftTurn() {
    _motorDriver.stop();
    Orientation before = _orientation;
    _orientation = turnLeft(_orientation);
    Serial.printf("ROTATION FINISHED: left from=%s to=%s\n",
                  orientationName(before),
                  orientationName(_orientation));
}

void MovementController::finishRightTurn() {
    _motorDriver.stop();
    Orientation before = _orientation;
    _orientation = turnRight(_orientation);
    Serial.printf("ROTATION FINISHED: right from=%s to=%s\n",
                  orientationName(before),
                  orientationName(_orientation));
}

void MovementController::advancePosition() {
    switch (_orientation) {
        case Orientation::NORTH:
            --_position.y;
            break;
        case Orientation::SOUTH:
            ++_position.y;
            break;
        case Orientation::EAST:
            ++_position.x;
            break;
        case Orientation::WEST:
            --_position.x;
            break;
    }

    if (_position.x < 0 || _position.y < 0 ||
        _position.x >= navigation::GridMap::Width ||
        _position.y >= navigation::GridMap::Height) {
        Serial.printf("MOVEMENT WARNING: position outside grid after advance (%d,%d)\n", _position.x, _position.y);
    }
}

bool MovementController::elapsed(unsigned long currentMillis, unsigned long durationMillis) {
    if (_stateStartedAt == 0) {
        _stateStartedAt = currentMillis;
        return durationMillis == 0;
    }

    return currentMillis - _stateStartedAt >= durationMillis;
}

Orientation MovementController::orientationForDelta(int16_t deltaX, int16_t deltaY) const {
    if (deltaX > 0) {
        return Orientation::EAST;
    }

    if (deltaX < 0) {
        return Orientation::WEST;
    }

    if (deltaY < 0) {
        return Orientation::NORTH;
    }

    return Orientation::SOUTH;
}

uint8_t MovementController::rightTurnsTo(Orientation targetOrientation) const {
    Orientation current = _orientation;

    for (uint8_t turns = 0; turns < 4; ++turns) {
        if (current == targetOrientation) {
            return turns;
        }

        current = turnRight(current);
    }

    return 0;
}

Orientation MovementController::turnLeft(Orientation orientation) {
    switch (orientation) {
        case Orientation::NORTH:
            return Orientation::WEST;
        case Orientation::SOUTH:
            return Orientation::EAST;
        case Orientation::EAST:
            return Orientation::NORTH;
        case Orientation::WEST:
            return Orientation::SOUTH;
    }

    return Orientation::SOUTH;
}

Orientation MovementController::turnRight(Orientation orientation) {
    switch (orientation) {
        case Orientation::NORTH:
            return Orientation::EAST;
        case Orientation::SOUTH:
            return Orientation::WEST;
        case Orientation::EAST:
            return Orientation::SOUTH;
        case Orientation::WEST:
            return Orientation::NORTH;
    }

    return Orientation::SOUTH;
}

const char* MovementController::orientationName(Orientation orientation) {
    switch (orientation) {
        case Orientation::NORTH:
            return "NORTH";
        case Orientation::SOUTH:
            return "SOUTH";
        case Orientation::EAST:
            return "EAST";
        case Orientation::WEST:
            return "WEST";
    }

    return "UNKNOWN";
}

const char* MovementController::stateName(State state) {
    switch (state) {
        case State::Idle:
            return "Idle";
        case State::MovingForward:
            return "MovingForward";
        case State::TurningLeft:
            return "TurningLeft";
        case State::TurningRight:
            return "TurningRight";
        case State::Pausing:
            return "Pausing";
    }

    return "Unknown";
}

void MovementController::logState(State nextState) {
    if (_state != nextState) {
        Serial.printf("MOVEMENT STATE -> %s\n", stateName(nextState));
    }

    _state = nextState;
}

} // namespace movement
