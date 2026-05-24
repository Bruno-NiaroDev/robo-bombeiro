#include "movement/MovementController.h"
#include "navigation/GridMap.h"
#include <Arduino.h>
#include <math.h>

namespace movement {

// ── Construtor ───────────────────────────────────────────────────────────────
//
// Os ganhos padrão são pontos de partida adequados para um robô diferencial
// com L298N a ~7,4 V e motores N20 / TT-Gear de 3-6 V.
// Ajuste via setTurnPid() / setHeadingPid() após carregar a configuração.
//
// _turnPid   kp=2.5  ki=0.2  kd=0.5  out=[0, 255]
//   - kp=2.5 → a 90° restantes, P=225 (≈ speed máxima).
//   - kd=0.5 → amorte-ce a aproximação final, reduz overshoot.
//   - ki=0.2 → pequeno integral para superar atrito estático nos últimos graus.
//
// _headingPid  kp=1.8  ki=0.05  kd=0.02  out=[-25, +25]
//   - kp=1.8 → a 3° de drift, correção ≈ ±5 PWM (sutil, não oscila).
//   - ki=0.05 → compensa deriva sistemática (motor mais forte, declive suave).
//   - kd=0.02 → amortece oscilações de heading em superfícies escorregadias.

MovementController::MovementController(drivers::MotorDriver& motorDriver)
    : _motorDriver(motorDriver)
    , _turnPid(2.5f, 0.2f, 0.5f, 0.0f, 255.0f)
    , _headingPid(1.8f, 0.05f, 0.02f, -25.0f, 25.0f)
{}

// ── Configuração ─────────────────────────────────────────────────────────────

void MovementController::setImu(drivers::ImuDriver* imu) {
    _imu = imu;
    Serial.println(imu ? "MOVEMENT: IMU conectado" : "MOVEMENT: sem IMU (modo timer)");
}

void MovementController::setTurnPid(float kp, float ki, float kd) {
    _turnPid.setGains(kp, ki, kd);
    Serial.printf("MOVEMENT: turnPid kp=%.3f ki=%.3f kd=%.3f\n", kp, ki, kd);
}

void MovementController::setHeadingPid(float kp, float ki, float kd) {
    _headingPid.setGains(kp, ki, kd);
    Serial.printf("MOVEMENT: headingPid kp=%.3f ki=%.3f kd=%.3f\n", kp, ki, kd);
}

void MovementController::setMinTurnSpeed(uint8_t speed) {
    _minTurnSpeed = speed;
}

void MovementController::setTurnTolerance(float toleranceDeg) {
    _turnToleranceDeg = toleranceDeg;
}

void MovementController::setTurnLeftPositiveYaw(bool positive) {
    _turnLeftPositiveYaw = positive;
}

void MovementController::resetImuReference() {
    if (_imu && _imu->isReady()) {
        _imu->resetYaw();
    }
    _startHeading = 0.0f;
    _startYaw = 0.0f;
}

float MovementController::turnedAmount(bool rightTurn) const {
    if (!_imu || !_imu->isReady()) {
        return 0.0f;
    }

    float yaw = _imu->yaw();
    if (!_turnLeftPositiveYaw) {
        yaw = -yaw;
    }

  // Esquerda: yaw positivo. Direita: yaw negativo → progresso = -yaw.
    return rightTurn ? -yaw : yaw;
}

void MovementController::begin() {
    _position     = {0, 0};
    _orientation  = Orientation::SOUTH;
    _state        = State::Idle;
    _pendingAction = PendingAction::None;
    _stateStartedAt    = 0;
    _lastUpdateMillis  = 0;
    _dt                = 0.0f;
    _motorDriver.stop();
}

// ── Loop principal ────────────────────────────────────────────────────────────
//
// Estrutura intencional:
//   1. Calcula dt para os PIDs
//   2. Aplica o estado base dos motores (dirty flag do MotorDriver)
//   3. O case ativo sobrescreve com a saída PID, se necessário
//
// A aplicação do estado base antes do case garante que o MotorDriver processe
// sempre o último comando, mesmo quando o PID não muda o valor.

void MovementController::update(unsigned long currentMillis) {
    // ── Delta-time para os PIDs ──────────────────────────────────────────────
    _dt = 0.0f;
    if (_lastUpdateMillis != 0) {
        unsigned long deltaMs = currentMillis - _lastUpdateMillis;
        if (deltaMs > 100) deltaMs = 100;   // cap: evita salto de integral após pausa longa
        _dt = deltaMs * 0.001f;             // ms → s
    }
    _lastUpdateMillis = currentMillis;

    // ── Aplica estado base primeiro; PIDs vêm a seguir ───────────────────────
    _motorDriver.update(currentMillis);

    switch (_state) {
        case State::Idle:
            break;

        case State::MovingForward:
            if (_imu && _imu->isReady()) {
                applyHeadingCorrection();
            } else {
                _motorDriver.forward(currentMoveSpeed());
            }
            if (elapsed(currentMillis, _moveDurationMillis)) {
                finishForwardMove();
                startPause(PendingAction::None);
            }
            break;

        case State::MovingBackward:
            if (_imu && _imu->isReady()) {
                applyBackwardHeadingCorrection();
            } else {
                _motorDriver.backward(currentMoveSpeed());
            }
            if (elapsed(currentMillis, _moveDurationMillis)) {
                finishBackwardMove();
                startPause(PendingAction::None);
            }
            break;

        case State::TurningLeft:
            applyTurnControl(false);
            if (turnComplete(currentMillis)) {
                finishLeftTurn();
                startPause(_pendingAction);
            }
            break;

        case State::TurningRight:
            applyTurnControl(true);
            if (turnComplete(currentMillis)) {
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
                        startForwardMove();
                        break;
                    case PendingAction::TurnLeftThenMoveBackward:
                        startBackwardMove();
                        break;
                    case PendingAction::ChainRightTurnsThenForward:
                    case PendingAction::ChainRightTurnsThenBackward:
                        // Passa nextAction explicitamente: _pendingAction já é None aqui.
                        continueTurnSequence(nextAction);
                        break;
                    case PendingAction::MoveForward:
                        startForwardMove();
                        break;
                    case PendingAction::MoveBackward:
                        startBackwardMove();
                        break;
                }
            }
            break;
    }
}

// ── Comandos públicos ─────────────────────────────────────────────────────────

bool MovementController::moveForwardOneCell() {
    if (_state != State::Idle) {
        Serial.printf("MOVEMENT REJECTED: busy state=%s\n", stateName(_state));
        return false;
    }
    Serial.printf("MOVEMENT COMMAND: forward from (%d,%d) orientation=%s\n",
                  _position.x, _position.y, orientationName(_orientation));
    return startForwardMove();
}

bool MovementController::moveBackwardOneCell() {
    if (_state != State::Idle) {
        Serial.printf("REVERSE REJECTED: busy state=%s\n", stateName(_state));
        return false;
    }

    Serial.printf("REVERSE COMMAND: straight back from (%d,%d) orientation=%s\n",
                  _position.x,
                  _position.y,
                  orientationName(_orientation));
    return startBackwardMove();
}

bool MovementController::moveToAdjacentCell(int16_t targetX, int16_t targetY) {
    if (_state != State::Idle) {
        Serial.printf("MOVEMENT REJECTED: busy state=%s target=(%d,%d)\n",
                      stateName(_state), targetX, targetY);
        return false;
    }

    Serial.printf("MOVEMENT TARGET: current=(%d,%d) next=(%d,%d) orientation=%s\n",
                  _position.x, _position.y, targetX, targetY, orientationName(_orientation));

    if (targetX < 0 || targetY < 0 ||
        targetX >= navigation::GridMap::Width ||
        targetY >= navigation::GridMap::Height) {
        Serial.printf("MOVEMENT REJECTED: target outside grid (%d,%d)\n", targetX, targetY);
        return false;
    }

    const int16_t deltaX = targetX - _position.x;
    const int16_t deltaY = targetY - _position.y;

    if ((deltaX == 0 && deltaY == 0) || (deltaX != 0 && deltaY != 0)) {
        Serial.printf("MOVEMENT REJECTED: invalid delta dx=%d dy=%d\n", deltaX, deltaY);
        return false;
    }
    if (deltaX < -1 || deltaX > 1 || deltaY < -1 || deltaY > 1) {
        Serial.printf("MOVEMENT REJECTED: not adjacent dx=%d dy=%d\n", deltaX, deltaY);
        return false;
    }

    const Orientation targetOrientation = orientationForDelta(deltaX, deltaY);
    const uint8_t turns = rightTurnsTo(targetOrientation);
    Serial.printf("MOVEMENT PLAN: targetOrientation=%s rightTurns=%u\n",
                  orientationName(targetOrientation), turns);

    return startTurnSequence(turns, PendingAction::MoveForward);
}

bool MovementController::moveBackwardToAdjacentCell(int16_t targetX, int16_t targetY) {
    if (_state != State::Idle) {
        Serial.printf("REVERSE REJECTED: busy state=%s target=(%d,%d)\n",
                      stateName(_state), targetX, targetY);
        return false;
    }

    Serial.printf("REVERSE TARGET: current=(%d,%d) next=(%d,%d) orientation=%s\n",
                  _position.x, _position.y, targetX, targetY, orientationName(_orientation));

    if (targetX < 0 || targetY < 0 ||
        targetX >= navigation::GridMap::Width ||
        targetY >= navigation::GridMap::Height) {
        Serial.printf("REVERSE REJECTED: target outside grid (%d,%d)\n", targetX, targetY);
        return false;
    }

    const int16_t deltaX = targetX - _position.x;
    const int16_t deltaY = targetY - _position.y;

    if ((deltaX == 0 && deltaY == 0) || (deltaX != 0 && deltaY != 0)) {
        Serial.printf("REVERSE REJECTED: invalid delta dx=%d dy=%d\n", deltaX, deltaY);
        return false;
    }
    if (deltaX < -1 || deltaX > 1 || deltaY < -1 || deltaY > 1) {
        Serial.printf("REVERSE REJECTED: not adjacent dx=%d dy=%d\n", deltaX, deltaY);
        return false;
    }

    int16_t behindX = _position.x;
    int16_t behindY = _position.y;
    switch (_orientation) {
        case Orientation::NORTH:
            ++behindY;
            break;
        case Orientation::SOUTH:
            --behindY;
            break;
        case Orientation::EAST:
            --behindX;
            break;
        case Orientation::WEST:
            ++behindX;
            break;
    }

    if (targetX == behindX && targetY == behindY) {
        Serial.printf("REVERSE: straight back to (%d,%d) without turn\n", targetX, targetY);
        return startBackwardMove();
    }

    const Orientation targetDirection    = orientationForDelta(deltaX, deltaY);
    const Orientation reverseOrientation = opposite(targetDirection);
    const uint8_t turns = rightTurnsTo(reverseOrientation);
    Serial.printf("REVERSE PLAN: rearDir=%s desiredOrientation=%s rightTurns=%u (realign)\n",
                  orientationName(targetDirection),
                  orientationName(reverseOrientation),
                  turns);

    return startTurnSequence(turns, PendingAction::MoveBackward);
}

bool MovementController::turnLeft90() {
    if (_state != State::Idle) {
        Serial.printf("TURN LEFT REJECTED: busy state=%s\n", stateName(_state));
        return false;
    }
    return startLeftTurn();
}

bool MovementController::turnRight90() {
    if (_state != State::Idle) {
        Serial.printf("TURN RIGHT REJECTED: busy state=%s\n", stateName(_state));
        return false;
    }
    return startRightTurn();
}

void MovementController::stop() {
    _motorDriver.stop();
    logState(State::Idle);
    _pendingAction = PendingAction::None;
    _turnPid.reset();
    _headingPid.reset();
    Serial.printf("MOVEMENT STOP: position=(%d,%d) orientation=%s\n",
                  _position.x, _position.y, orientationName(_orientation));
}

// ── Consultas de estado ───────────────────────────────────────────────────────

bool MovementController::isBusy()  const { return _state != State::Idle; }
bool MovementController::isPaused() const { return _state == State::Pausing; }

bool MovementController::isMoving() const {
    return _state == State::MovingForward  ||
           _state == State::MovingBackward ||
           _state == State::TurningLeft    ||
           _state == State::TurningRight;
}

GridPosition MovementController::position()    const { return _position; }
Orientation  MovementController::orientation() const { return _orientation; }

// ── Configuração de posição / orientação ──────────────────────────────────────

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

void MovementController::setMoveDuration(unsigned long d)  { _moveDurationMillis  = d; }
void MovementController::setTurnDuration(unsigned long d)  { _turnDurationMillis  = d; }
void MovementController::setPauseDuration(unsigned long d) { _pauseDurationMillis = d; }
void MovementController::setMoveSpeed(uint8_t s)           { _moveSpeed           = s; }
void MovementController::setTurnSpeed(uint8_t s)           { _turnSpeed           = s; }

// ── Início de movimentos ──────────────────────────────────────────────────────

bool MovementController::startForwardMove() {
    resetImuReference();
    _headingPid.reset();
    _motorDriver.forward(_moveSpeed);
    logState(State::MovingForward);
    _stateStartedAt = 0;
    Serial.printf("MOVEMENT EXECUTED: forward speed=%u orientation=%s imu=%s\n",
                  _moveSpeed,
                  orientationName(_orientation),
                  (_imu && _imu->isReady()) ? "ok" : "timer");
    return true;
}

bool MovementController::startBackwardMove() {
    resetImuReference();
    _headingPid.reset();
    _motorDriver.backward(_moveSpeed);
    logState(State::MovingBackward);
    _stateStartedAt = 0;
    Serial.printf("MOVEMENT EXECUTED: backward speed=%u orientation=%s imu=%s\n",
                  _moveSpeed,
                  orientationName(_orientation),
                  (_imu && _imu->isReady()) ? "ok" : "timer");
    return true;
}

bool MovementController::startLeftTurn() {
    resetImuReference();
    _turnPid.reset();
    _motorDriver.turnLeft(_turnSpeed);
    logState(State::TurningLeft);
    _stateStartedAt = 0;
    Serial.printf("ROTATION EXECUTED: left speed=%u from=%s imu=%s\n",
                  _turnSpeed,
                  orientationName(_orientation),
                  (_imu && _imu->isReady()) ? "ok" : "timer");
    return true;
}

bool MovementController::startRightTurn() {
    resetImuReference();
    _turnPid.reset();
    _motorDriver.turnRight(_turnSpeed);
    logState(State::TurningRight);
    _stateStartedAt = 0;
    Serial.printf("ROTATION EXECUTED: right speed=%u from=%s imu=%s\n",
                  _turnSpeed,
                  orientationName(_orientation),
                  (_imu && _imu->isReady()) ? "ok" : "timer");
    return true;
}

void MovementController::startPause(PendingAction nextAction) {
    _motorDriver.stop();
    _pendingAction = nextAction;
    logState(State::Pausing);
    _stateStartedAt = 0;
}

bool MovementController::startTurnSequence(uint8_t rightTurnCount, PendingAction finalMove) {
    if (rightTurnCount == 0) {
        return finalMove == PendingAction::MoveForward ? startForwardMove() : startBackwardMove();
    }

    if (rightTurnCount == 3) {
        _pendingAction = finalMove == PendingAction::MoveForward
            ? PendingAction::TurnLeftThenMove
            : PendingAction::TurnLeftThenMoveBackward;
        return startLeftTurn();
    }

    _remainingRightTurns = rightTurnCount - 1;
    _pendingAction = finalMove == PendingAction::MoveForward
        ? PendingAction::ChainRightTurnsThenForward
        : PendingAction::ChainRightTurnsThenBackward;
    return startRightTurn();
}

void MovementController::continueTurnSequence(PendingAction chainAction) {
    if (_remainingRightTurns > 0) {
        --_remainingRightTurns;
        // Restaura _pendingAction para que finishRightTurn → startPause saiba
        // continuar o encadeamento na próxima iteração do Pausing.
        _pendingAction = chainAction;
        startRightTurn();
        return;
    }

    // Todos os giros concluídos — executa o movimento final.
    if (chainAction == PendingAction::ChainRightTurnsThenForward) {
        startForwardMove();
    } else {
        startBackwardMove();
    }
}

uint8_t MovementController::currentMoveSpeed() const {
    if (_stateStartedAt == 0 || _moveDurationMillis == 0) {
        return _moveSpeed;
    }

    const unsigned long elapsedMs = _lastUpdateMillis - _stateStartedAt;
    const unsigned long rampStart = (_moveDurationMillis * 3) / 4;

    if (elapsedMs <= rampStart) {
        return _moveSpeed;
    }

    const unsigned long rampDuration = _moveDurationMillis - rampStart;
    if (rampDuration == 0) {
        return _moveSpeed;
    }

    const unsigned long inRamp = elapsedMs - rampStart;
    const float ratio = 1.0f - (static_cast<float>(inRamp) / static_cast<float>(rampDuration));
    const uint8_t speed = static_cast<uint8_t>(static_cast<float>(_moveSpeed) * ratio);
    return speed < 40 ? 40 : speed;
}

// ── Finalização de movimentos ─────────────────────────────────────────────────

void MovementController::finishForwardMove() {
    _motorDriver.stop();
    advancePosition();
    Serial.printf("MOVEMENT FINISHED: position=(%d,%d) orientation=%s\n",
                  _position.x, _position.y, orientationName(_orientation));
}

void MovementController::finishBackwardMove() {
    _motorDriver.stop();
    retreatPosition();
    Serial.printf("MOVEMENT FINISHED: reverse position=(%d,%d) orientation=%s\n",
                  _position.x, _position.y, orientationName(_orientation));
}

void MovementController::finishLeftTurn() {
    _motorDriver.stop();
    const Orientation before = _orientation;
    const float turned = turnedAmount(false);
    _orientation = turnLeft(_orientation);
    resetImuReference();
    Serial.printf("ROTATION FINISHED: left from=%s to=%s turned=%.2f\n",
                  orientationName(before),
                  orientationName(_orientation),
                  turned);
}

void MovementController::finishRightTurn() {
    _motorDriver.stop();
    const Orientation before = _orientation;
    const float turned = turnedAmount(true);
    _orientation = turnRight(_orientation);
    resetImuReference();
    Serial.printf("ROTATION FINISHED: right from=%s to=%s turned=%.2f\n",
                  orientationName(before),
                  orientationName(_orientation),
                  turned);
}

// ── Controle de curva — PID de ângulo ─────────────────────────────────────────
//
// A cada iteração do loop, calcula:
//   error = (90° − tolerância) − ângulo_girado
//
// O PID mapeia esse erro em uma velocidade PWM.
// Um floor em _minTurnSpeed garante que os motores continuem se movendo
// mesmo quando o ganho proporcional produziria saída abaixo da zona morta
// eletromecânica do motor (tipicamente 40–70 PWM para L298N).
//
// Quando error ≤ 0 o método retorna sem acionar; turnComplete() cuida do stop.

void MovementController::applyTurnControl(bool rightTurn) {
    if (!_imu || !_imu->isReady() || _dt <= 0.0f) {
        return;
    }

    const float turned = turnedAmount(rightTurn);
    const float target = 90.0f - _turnToleranceDeg;
    const float error  = target - turned;

    if (error <= 0.0f) {
        _motorDriver.stop();
        return;
    }

    const float pidOut = _turnPid.compute(error, _dt);

    // Reduz PWM mínimo nos últimos graus para evitar overshoot por inércia
    uint8_t minSpeed = _minTurnSpeed;
    if (error < 20.0f) {
        const float scale = error / 20.0f;
        minSpeed = static_cast<uint8_t>(static_cast<float>(_minTurnSpeed) * scale);
    }

    const uint8_t speed = static_cast<uint8_t>(
        pidOut < static_cast<float>(minSpeed) ? minSpeed :
        pidOut > static_cast<float>(_turnSpeed)    ? _turnSpeed    :
        static_cast<uint8_t>(pidOut)
    );

    if (rightTurn) {
        _motorDriver.turnRight(speed);
    } else {
        _motorDriver.turnLeft(speed);
    }
}

// ── Condição de fim de curva ──────────────────────────────────────────────────
//
// Usa o ângulo da IMU como critério primário.
// Timer de segurança (1,5× turnDuration) evita travar se a IMU falhar ou
// se o robô parar mecanicamente antes de atingir o ângulo.

bool MovementController::turnComplete(unsigned long currentMillis) {
    const unsigned long safetyTimeout = _turnDurationMillis + (_turnDurationMillis >> 1);

    if (_imu && _imu->isReady()) {
        const bool rightTurn = _state == State::TurningRight;
        const float turned   = turnedAmount(rightTurn);
        const float target   = 90.0f - _turnToleranceDeg;
        const bool done      = turned >= target;
        const bool timedOut  = elapsed(currentMillis, safetyTimeout);

        if (done) {
            Serial.printf("TURN DONE  (IMU): turned=%.2f target=%.2f\n", turned, target);
            return true;
        }

        if (timedOut) {
            Serial.printf("TURN TIMEOUT (IMU): turned=%.2f target=%.2f — verifique IMU/motores\n",
                          turned,
                          target);
            return true;
        }

        return false;
    }

    // Sem IMU: usa timer calibrado (nao 1,5x) para evitar curvas exageradas
    if (elapsed(currentMillis, _turnDurationMillis)) {
        Serial.println("TURN DONE (TIMER): IMU indisponivel");
        return true;
    }

    return false;
}

// ── Controle de heading — PID de deriva ───────────────────────────────────────
//
// Durante movimento reto o robô pode derivar por assimetria dos motores,
// superfície irregular ou variação de tensão da bateria.
// O PID calcula uma correção diferencial (PWM) para manter o heading inicial.
//
// Convenção de sinal (driveWithCorrection):
//   correction > 0 → motor esquerdo mais lento, direito mais rápido → robô vira CCW
//   correction < 0 → motor esquerdo mais rápido, direito mais lento → robô vira CW
//
// Portanto:
//   yaw aumenta (deriva CW)  → error > 0 → PID output > 0 → correção CCW ✓
//   yaw diminui (deriva CCW) → error < 0 → PID output < 0 → correção CW  ✓

void MovementController::applyHeadingCorrection() {
    if (!_imu || !_imu->isReady() || _dt <= 0.0f) return;

    const float error = _imu->yaw() - _startHeading;
    if (fabsf(error) < kHeadingDeadbandDeg) {
        _motorDriver.forward(currentMoveSpeed());
        return;
    }

    const float correction = _headingPid.compute(error, _dt);
    _motorDriver.driveWithCorrection(currentMoveSpeed(), static_cast<int8_t>(correction));
}

void MovementController::applyBackwardHeadingCorrection() {
    if (!_imu || !_imu->isReady() || _dt <= 0.0f) return;

    const float error = _imu->yaw() - _startHeading;
    if (fabsf(error) < kHeadingDeadbandDeg) {
        _motorDriver.backward(currentMoveSpeed());
        return;
    }

    const float correction = _headingPid.compute(error, _dt);
    _motorDriver.driveBackwardWithCorrection(currentMoveSpeed(), static_cast<int8_t>(correction));
}

// ── Odometria ────────────────────────────────────────────────────────────────

void MovementController::advancePosition() {
    switch (_orientation) {
        case Orientation::NORTH: --_position.y; break;
        case Orientation::SOUTH: ++_position.y; break;
        case Orientation::EAST:  ++_position.x; break;
        case Orientation::WEST:  --_position.x; break;
    }
    if (_position.x < 0 || _position.y < 0 ||
        _position.x >= navigation::GridMap::Width ||
        _position.y >= navigation::GridMap::Height) {
        Serial.printf("MOVEMENT WARNING: position outside grid after advance (%d,%d)\n",
                      _position.x, _position.y);
    }
}

void MovementController::retreatPosition() {
    switch (_orientation) {
        case Orientation::NORTH: ++_position.y; break;
        case Orientation::SOUTH: --_position.y; break;
        case Orientation::EAST:  --_position.x; break;
        case Orientation::WEST:  ++_position.x; break;
    }
    if (_position.x < 0 || _position.y < 0 ||
        _position.x >= navigation::GridMap::Width ||
        _position.y >= navigation::GridMap::Height) {
        Serial.printf("MOVEMENT WARNING: position outside grid after reverse (%d,%d)\n",
                      _position.x, _position.y);
    }
}

// ── Timer de estado ───────────────────────────────────────────────────────────

bool MovementController::elapsed(unsigned long currentMillis, unsigned long durationMillis) {
    if (_stateStartedAt == 0) {
        _stateStartedAt = currentMillis;
        return durationMillis == 0;
    }
    return currentMillis - _stateStartedAt >= durationMillis;
}

// ── Geometria de grid ─────────────────────────────────────────────────────────

Orientation MovementController::orientationForDelta(int16_t deltaX, int16_t deltaY) const {
    if (deltaX > 0) return Orientation::EAST;
    if (deltaX < 0) return Orientation::WEST;
    if (deltaY < 0) return Orientation::NORTH;
    return Orientation::SOUTH;
}

uint8_t MovementController::rightTurnsTo(Orientation targetOrientation) const {
    Orientation current = _orientation;
    for (uint8_t turns = 0; turns < 4; ++turns) {
        if (current == targetOrientation) return turns;
        current = turnRight(current);
    }
    return 0;
}

// ── Utilitários de orientação ─────────────────────────────────────────────────

Orientation MovementController::opposite(Orientation o) {
    switch (o) {
        case Orientation::NORTH: return Orientation::SOUTH;
        case Orientation::SOUTH: return Orientation::NORTH;
        case Orientation::EAST:  return Orientation::WEST;
        case Orientation::WEST:  return Orientation::EAST;
    }
    return Orientation::SOUTH;
}

Orientation MovementController::turnLeft(Orientation o) {
    switch (o) {
        case Orientation::NORTH: return Orientation::WEST;
        case Orientation::SOUTH: return Orientation::EAST;
        case Orientation::EAST:  return Orientation::NORTH;
        case Orientation::WEST:  return Orientation::SOUTH;
    }
    return Orientation::SOUTH;
}

Orientation MovementController::turnRight(Orientation o) {
    switch (o) {
        case Orientation::NORTH: return Orientation::EAST;
        case Orientation::SOUTH: return Orientation::WEST;
        case Orientation::EAST:  return Orientation::SOUTH;
        case Orientation::WEST:  return Orientation::NORTH;
    }
    return Orientation::SOUTH;
}

const char* MovementController::orientationName(Orientation o) {
    switch (o) {
        case Orientation::NORTH: return "NORTH";
        case Orientation::SOUTH: return "SOUTH";
        case Orientation::EAST:  return "EAST";
        case Orientation::WEST:  return "WEST";
    }
    return "UNKNOWN";
}

const char* MovementController::stateName(State s) {
    switch (s) {
        case State::Idle:           return "Idle";
        case State::MovingForward:  return "MovingForward";
        case State::MovingBackward: return "MovingBackward";
        case State::TurningLeft:    return "TurningLeft";
        case State::TurningRight:   return "TurningRight";
        case State::Pausing:        return "Pausing";
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
