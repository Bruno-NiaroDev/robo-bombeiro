#include "control/PidController.h"

namespace control {

PidController::PidController(float kp, float ki, float kd, float outMin, float outMax)
    : _kp(kp), _ki(ki), _kd(kd), _outMin(outMin), _outMax(outMax) {}

float PidController::compute(float error, float dt) {
    if (dt <= 0.0f) return 0.0f;

    // ── Proporcional ─────────────────────────────────────────────────────────
    const float pOut = _kp * error;

    // ── Integral com anti-windup por clamping ─────────────────────────────────
    // A integral acumula o erro ao longo do tempo.
    // Anti-windup: limitamos _integral para que ki*integral jamais exceda
    // os limites de saída, evitando resposta lenta após saturação prolongada.
    _integral += error * dt;
    if (_ki != 0.0f) {
        const float intMax = _outMax / _ki;
        const float intMin = _outMin / _ki;
        // normaliza bounds independentemente do sinal de ki
        if (_integral > intMax && intMax > intMin) _integral = intMax;
        if (_integral < intMin && intMax > intMin) _integral = intMin;
        if (_integral < intMax && intMax < intMin) _integral = intMax;
        if (_integral > intMin && intMax < intMin) _integral = intMin;
    }
    const float iOut = _ki * _integral;

    // ── Derivativo baseado no erro (não na variável de processo) ──────────────
    // Calcular a derivada do erro em vez da da medição evita o "derivative kick"
    // que ocorre quando o setpoint muda abruptamente. Na prática: o D amorte-
    // ce a aproximação ao alvo e reduz overshoot nas curvas.
    float dOut = 0.0f;
    if (!_firstCall) {
        dOut = _kd * (error - _prevError) / dt;
    }
    _firstCall = false;
    _prevError = error;

    // ── Saída com clamping ────────────────────────────────────────────────────
    float out = pOut + iOut + dOut;
    if (out > _outMax) out = _outMax;
    if (out < _outMin) out = _outMin;
    return out;
}

void PidController::reset() {
    _integral  = 0.0f;
    _prevError = 0.0f;
    _firstCall = true;
}

void PidController::setGains(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
    reset();
}

void PidController::setOutputLimits(float outMin, float outMax) {
    _outMin = outMin;
    _outMax = outMax;
}

} // namespace control
