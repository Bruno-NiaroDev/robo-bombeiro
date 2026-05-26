#pragma once

#include <stdint.h>
#include "SensorBase.h"

namespace sensors {

enum class FireDirection {
    None,
    Left,
    Center,
    Right
};

/// Scanner de chama com dois servos (horizontal + vertical) e sensor E27-004.
///
/// Lógica de posição:
///   - Repouso  (begin / center): horizontal = 0°,  vertical = 0°
///   - Busca (enableSweep(true)): vertical sobe para _verticalSearchAngle (padrão 60°)
///                                 horizontal varre de _minAngle até _maxAngle
///
/// Detecção (sensor E27-004 analógico, 4 pinos):
///   - Sem baseline, sem calibração — comparação direta de threshold
///   - Comportamento observado neste módulo:
///     * Sem chama: ADC natural baixo (~110)
///     * Com chama: sinal sobe (fotodiodo gera corrente → A0 aumenta)
///   - Padrão: valor ALTO = chama (_activeWhenLow = false, threshold = 500)
///   - Se o seu módulo funciona ao contrário: chame setSensorActiveWhenLow(true)
///   - Ajuste setFireThreshold() com base no campo "flameRaw" da telemetria WebSocket
class FireScanner : public SensorBase {
public:
    FireScanner();
    FireScanner(int horizontalServoPin, int verticalServoPin, int flameSensorPin);

    void begin() override;
    void update(unsigned long currentMillis) override;
    bool hasNewData() const override;

    void clearNewData();

    bool          fireDetected()   const;
    FireDirection fireDirection()  const;
    uint8_t       fireAngle()      const;
    uint8_t       currentAngle()   const;
    uint16_t      rawValue()       const; ///< Última leitura ADC do sensor de chama (0–4095).

    // ── Configuração de varredura horizontal ─────────────────────────────────

    /// Define o intervalo de varredura horizontal (padrão: 0°–180°).
    void setSweepRange(uint8_t minAngle, uint8_t maxAngle);
    /// Ângulo central usado apenas para classificação Left/Center/Right.
    void setDirectionCenterAngle(uint8_t centerAngle);
    /// Intervalo entre cada passo de varredura (padrão: 20 ms).
    void setStepInterval(unsigned long intervalMillis);
    /// Graus por passo de varredura (padrão: 2°).
    void setStepDegrees(uint8_t stepDegrees);

    // ── Configuração do servo vertical ───────────────────────────────────────

    /// Ângulo do servo vertical durante busca de fogo (padrão: 100°).
    void setVerticalSearchAngle(uint8_t angle);
    /// Ângulo de repouso do servo vertical (padrão: 0°).
    void setVerticalRestAngle(uint8_t angle);
    /// Pulso mínimo e máximo do servo vertical em µs.
    void setVerticalServoPulseRange(uint16_t minPulseMicros, uint16_t maxPulseMicros);

    // ── Configuração da detecção ─────────────────────────────────────────────

    /// Threshold ADC para detecção (escala 0–4095, 12 bits). Usado apenas no modo analógico.
    /// Padrão: 500 — sinal ambiente ~110, ajuste com base no flameRaw da telemetria.
    void setFireThreshold(uint16_t threshold);

    /// true  = valor BAIXO indica chama (D0 digital ativo-baixo, ou A0 cai com IR).
    /// false = valor ALTO  indica chama (A0 sobe com IR).
    void setSensorActiveWhenLow(bool activeWhenLow);

    /// Ativa leitura digital (pino D0 do E27-004) em vez de analógica (pino A0).
    /// No modo digital: LOW no pino = chama detectada (D0 é ativo-baixo).
    /// Use quando a saída A0 não gera sinal confiável.
    /// Padrão: false (modo analógico).
    void setDigitalMode(bool useDigital);

    /// Tempo de latch após o último sample positivo (padrão: 200 ms).
    /// Mantém a detecção ativa enquanto o servo varre além do ângulo da chama.
    void setLatchDuration(unsigned long millis);

    /// Tempo de settling após ativar detecção — leituras ignoradas nesse período (padrão: 500 ms).
    /// Evita falso positivo por ruído dos servos ou IR ambiente na posição inicial.
    void setSettlingDuration(unsigned long millis);

    /// Pulso mínimo e máximo do servo horizontal em µs.
    void setServoPulseRange(uint16_t minPulseMicros, uint16_t maxPulseMicros);

    // ── Controle de estado ───────────────────────────────────────────────────

    void enableDetection(bool enabled, unsigned long currentMillis);
    void enableSweep(bool enabled);
    bool sweepEnabled() const;

    /// Move horizontal para 0° e vertical para 0° (posição de repouso).
    void center();

private:
    void updateSweep(unsigned long currentMillis);
    void updateFireReading(unsigned long currentMillis);
    void resetDetection();
    void writeServoAngle(uint8_t angle);
    void writeVerticalAngle(uint8_t angle);
    uint16_t pulseForAngle(uint8_t angle, uint16_t minPulse, uint16_t maxPulse) const;
    FireDirection directionForAngle(uint8_t angle) const;
    uint8_t clampAngle(uint8_t angle) const;

    // ── Pinos ────────────────────────────────────────────────────────────────
    int _servoPin;
    int _verticalServoPin;
    int _flameSensorPin;

    // ── Canais PWM ───────────────────────────────────────────────────────────
    // Canais 0 e 1 reservados para motores (MotorDriver).
    static const uint8_t kHorizontalPwmChannel = 2;
    static const uint8_t kVerticalPwmChannel    = 3;

    // ── Estado dos servos ────────────────────────────────────────────────────
    uint8_t  _currentAngle          = 180; // repouso em 180° (extremidade)
    uint8_t  _verticalCurrentAngle  = 60;
    uint8_t  _verticalSearchAngle   = 60;
    uint8_t  _verticalRestAngle     = 60;

    // ── Parâmetros de varredura ──────────────────────────────────────────────
    uint8_t  _minAngle              = 50;
    uint8_t  _maxAngle              = 180;
    uint8_t  _directionCenterAngle  = 90;   // referência para Left/Center/Right
    uint8_t  _stepDegrees           = 2;
    bool     _sweepIncreasing       = true;
    unsigned long _stepIntervalMillis = 20;
    unsigned long _lastStepMillis    = 0;

    // ── Pulso dos servos (µs) ────────────────────────────────────────────────
    uint16_t _minPulseMicros         = 500;
    uint16_t _maxPulseMicros         = 2400;
    uint16_t _minPulseMicrosVertical = 500;
    uint16_t _maxPulseMicrosVertical = 2400;

    // ── Detecção de chama ────────────────────────────────────────────────────
    uint16_t     _fireThreshold           = 500;  // ADC 12 bits — usado só no modo analógico
    bool         _activeWhenLow           = false; // false = ADC alto indica chama (modo analógico)
    bool         _digitalMode             = true;  // true = lê D0 digitalmente (LOW = chama)
    unsigned long _latchDurationMillis        = 200; // mantém detecção após último sample positivo
    uint8_t       _consecutivePositiveCount   = 0;   // contagem de samples positivos consecutivos
    unsigned long _lastPositiveSampleMillis   = 0;   // referência do latch
    unsigned long _detectionEnabledAt         = 0;   // quando enableDetection(true) foi chamado
    unsigned long _settlingDurationMillis     = 300; // ignora leituras nos primeiros 300ms após ativar
    bool          _baselineEstablished        = false; // viu HIGH pelo menos uma vez pós-settling

    // ── Estado de detecção ───────────────────────────────────────────────────
    uint16_t      _lastRaw          = 0;   // última leitura ADC — exposta via rawValue()
    uint8_t       _fireAngle        = 0;
    bool          _fireDetected     = false;
    bool          _hasNewData       = false;
    bool          _detectionEnabled = false;
    bool          _sweepEnabled     = false;
    FireDirection _fireDirection    = FireDirection::None;
};

} // namespace sensors
