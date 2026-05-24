#pragma once

#include <stdint.h>

namespace control {

/// Controlador PID genérico, não-bloqueante.
///
/// Características:
///   - Anti-windup por clamping da integral
///   - Derivativo baseado no erro (não na PV) → sem "derivative kick"
///   - Saída clampada em [outMin, outMax]
///   - Sem `delay()` ou `millis()` internos; o caller passa dt
///
/// Uso típico:
///   PidController pid(kp, ki, kd, outMin, outMax);
///   pid.reset();
///   // em cada iteração do loop:
///   float saida = pid.compute(setpoint - medicao, dt_segundos);
///   acionar(saida);
class PidController {
public:
    /// @param kp      Ganho proporcional
    /// @param ki      Ganho integral
    /// @param kd      Ganho derivativo
    /// @param outMin  Limite inferior da saída (pode ser negativo)
    /// @param outMax  Limite superior da saída
    PidController(float kp, float ki, float kd, float outMin, float outMax);

    /// Calcula a próxima saída PID.
    /// @param error  Erro = setpoint − medição (positivo → saída positiva)
    /// @param dt     Intervalo desde a chamada anterior, em segundos (deve ser > 0)
    /// @return Saída clampada em [outMin, outMax]
    float compute(float error, float dt);

    /// Zera integral, erro anterior e flag de primeiro chamada.
    /// Chame sempre que iniciar um novo movimento para evitar transientes.
    void reset();

    /// Atualiza ganhos e reseta o estado interno.
    void setGains(float kp, float ki, float kd);

    /// Atualiza limites de saída.
    void setOutputLimits(float outMin, float outMax);

    // Accessors para diagnóstico / logging
    float kp()       const { return _kp; }
    float ki()       const { return _ki; }
    float kd()       const { return _kd; }
    float outMin()   const { return _outMin; }
    float outMax()   const { return _outMax; }
    float integral() const { return _integral; }

private:
    float _kp, _ki, _kd;
    float _outMin, _outMax;
    float _integral  = 0.0f;
    float _prevError = 0.0f;
    bool  _firstCall = true;
};

} // namespace control
