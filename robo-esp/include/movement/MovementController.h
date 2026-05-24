#pragma once

#include <stdint.h>
#include "drivers/MotorDriver.h"
#include "drivers/ImuDriver.h"
#include "control/PidController.h"

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

    // ── Comandos de movimento ────────────────────────────────────────────────

    bool moveForwardOneCell();
    bool moveToAdjacentCell(int16_t targetX, int16_t targetY);
    bool moveBackwardToAdjacentCell(int16_t targetX, int16_t targetY);
    /// Recua uma célula sem girar (usa orientação atual). Ideal para volta no mesmo caminho.
    bool moveBackwardOneCell();
    bool turnLeft90();
    bool turnRight90();
    void stop();

    // ── Consulta de estado ───────────────────────────────────────────────────

    bool isBusy()   const;
    bool isMoving() const;
    bool isPaused() const;

    GridPosition position()    const;
    Orientation  orientation() const;

    // ── Configuração de posição / orientação ─────────────────────────────────

    void setPosition(int16_t x, int16_t y);
    void setOrientation(Orientation orientation);

    // ── Parâmetros de movimento ──────────────────────────────────────────────

    void setMoveDuration(unsigned long durationMillis);
    void setTurnDuration(unsigned long durationMillis);
    void setPauseDuration(unsigned long durationMillis);
    void setMoveSpeed(uint8_t speed);
    void setTurnSpeed(uint8_t speed);

    // ── IMU e controle angular ───────────────────────────────────────────────

    void setImu(drivers::ImuDriver* imu);

    /// Define os ganhos do PID que controla a velocidade durante as curvas.
    /// Saída do PID = velocidade PWM em [minTurnSpeed, turnSpeed].
    void setTurnPid(float kp, float ki, float kd);

    /// Define os ganhos do PID de heading (correção de deriva em linha reta).
    /// Saída do PID = correção diferencial PWM em [-maxHeadingCorrection, +maxHeadingCorrection].
    void setHeadingPid(float kp, float ki, float kd);

    /// Velocidade PWM mínima durante curva — abaixo desse valor o motor não se move.
    /// Ajuste conforme a tensão da bateria e o atrito do robô (típico: 45–70).
    void setMinTurnSpeed(uint8_t speed);

    /// Graus antes de 90° em que o motor é desligado; a inércia completa o resto.
    /// Reduza se parar antes do alvo; aumente se ultrapassar (típico: 1.0–2.5°).
    void setTurnTolerance(float toleranceDeg);

    /// true se curva à esquerda aumenta yaw da IMU (padrão). false se motores/giro invertidos.
    void setTurnLeftPositiveYaw(bool positive);

private:
    // ── Estados internos ─────────────────────────────────────────────────────

    enum class State {
        Idle,
        MovingForward,
        MovingBackward,
        TurningLeft,
        TurningRight,
        Pausing
    };

    enum class PendingAction {
        None,
        TurnLeftThenMove,
        TurnLeftThenMoveBackward,
        ChainRightTurnsThenForward,
        ChainRightTurnsThenBackward,
        MoveBackward,
        MoveForward
    };

    // ── Métodos privados de controle ─────────────────────────────────────────

    bool startForwardMove();
    bool startBackwardMove();
    bool startLeftTurn();
    bool startRightTurn();
    void startPause(PendingAction nextAction);
    bool startTurnSequence(uint8_t rightTurnCount, PendingAction finalMove);
    // chainAction é o PendingAction original (Forward ou Backward), passado
    // explicitamente porque _pendingAction já foi zerado quando esta função é chamada.
    void continueTurnSequence(PendingAction chainAction);

    bool turnComplete(unsigned long currentMillis);

    /// Graus girados na curva atual (positivo = progresso em direção ao alvo de 90°).
    float turnedAmount(bool rightTurn) const;

    void resetImuReference();

    /// Controla a velocidade dos motores durante uma curva usando PID.
    /// Desacelera progressivamente ao se aproximar do ângulo-alvo.
    void applyTurnControl(bool rightTurn);

    /// Corrige deriva de heading durante avanço reto (usando PID).
    void applyHeadingCorrection();

    /// Corrige deriva de heading durante recuo reto (usando PID).
    void applyBackwardHeadingCorrection();

    uint8_t currentMoveSpeed() const;

    void finishForwardMove();
    void finishBackwardMove();
    void finishLeftTurn();
    void finishRightTurn();
    void advancePosition();
    void retreatPosition();

    bool elapsed(unsigned long currentMillis, unsigned long durationMillis);

    Orientation orientationForDelta(int16_t deltaX, int16_t deltaY) const;
    uint8_t     rightTurnsTo(Orientation targetOrientation) const;

    static Orientation  opposite(Orientation orientation);
    static Orientation  turnLeft(Orientation orientation);
    static Orientation  turnRight(Orientation orientation);
    static const char*  orientationName(Orientation orientation);
    static const char*  stateName(State state);
    void                logState(State nextState);

    // ── Constantes de controle ────────────────────────────────────────────────
    //
    // kHeadingDeadbandDeg: erros de heading abaixo deste valor são ignorados
    // para evitar que o PID "persiga" o ruído do giroscópio.
    static constexpr float kHeadingDeadbandDeg = 1.5f;

    // ── Dependências externas ────────────────────────────────────────────────

    drivers::MotorDriver& _motorDriver;
    drivers::ImuDriver*   _imu          = nullptr;

    // ── Controladores PID ────────────────────────────────────────────────────
    //
    // _turnPid:    entrada = graus restantes até 90°,  saída = velocidade PWM [0, 255]
    // _headingPid: entrada = erro de heading (graus),  saída = correção PWM  [-25, +25]

    control::PidController _turnPid;
    control::PidController _headingPid;

    // ── Estado de posição / orientação ────────────────────────────────────────

    GridPosition _position    = {0, 0};
    Orientation  _orientation = Orientation::SOUTH;
    State        _state       = State::Idle;
    PendingAction _pendingAction = PendingAction::None;
    uint8_t       _remainingRightTurns = 0;

    // ── Temporização ──────────────────────────────────────────────────────────

    unsigned long _stateStartedAt      = 0;
    unsigned long _lastUpdateMillis    = 0;
    float         _dt                  = 0.0f;  // segundos desde o último update()

    // ── Parâmetros configuráveis ──────────────────────────────────────────────

    unsigned long _moveDurationMillis  = 420;
    unsigned long _turnDurationMillis  = 1000;
    unsigned long _pauseDurationMillis = 180;
    uint8_t       _moveSpeed           = 150;
    uint8_t       _turnSpeed           = 250;
    uint8_t       _minTurnSpeed        = 55;    // PWM mínimo para vencer atrito estático

    // Graus antes de 90° em que o motor é cortado; inércia cobre o restante
    float         _turnToleranceDeg    = 1.5f;

    // ── Estado de navegação por IMU ───────────────────────────────────────────

    float _startYaw     = 0.0f;  // yaw no início de uma curva
    float _startHeading = 0.0f;  // heading no início de um avanço/recuo reto
    bool  _turnLeftPositiveYaw = true;
};

} // namespace movement
