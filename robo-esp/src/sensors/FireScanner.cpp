#include "sensors/FireScanner.h"
#include "Pins.h"
#include <Arduino.h>

namespace sensors {

static const uint16_t kServoFrequencyHz        = 50;
static const uint8_t  kServoPwmResolutionBits  = 16;
static const uint32_t kServoPeriodMicros       = 20000; // 1/50Hz = 20ms
static const uint32_t kMaxDuty = (1UL << kServoPwmResolutionBits) - 1;

// ── Construtores ──────────────────────────────────────────────────────────────

FireScanner::FireScanner()
    : FireScanner(pins::servo::HORIZONTAL, pins::servo::VERTICAL, pins::sensors::FLAME) {}

FireScanner::FireScanner(int horizontalServoPin, int verticalServoPin, int flameSensorPin)
    : _servoPin(horizontalServoPin)
    , _verticalServoPin(verticalServoPin)
    , _flameSensorPin(flameSensorPin) {}

// ── Inicialização ─────────────────────────────────────────────────────────────

void FireScanner::begin() {
    // Sensor de chama — configura pino conforme o modo (digital D0 ou analógico A0)
    if (_digitalMode) {
        pinMode(_flameSensorPin, INPUT_PULLUP); // D0 ativo-baixo; pull-up garante HIGH sem chama
    } else {
        pinMode(_flameSensorPin, INPUT);
        analogReadResolution(12);
        analogSetPinAttenuation(_flameSensorPin, ADC_11db);
    }

    // Servo horizontal
    ledcSetup(kHorizontalPwmChannel, kServoFrequencyHz, kServoPwmResolutionBits);
    ledcAttachPin(_servoPin, kHorizontalPwmChannel);

    // Servo vertical
    ledcSetup(kVerticalPwmChannel, kServoFrequencyHz, kServoPwmResolutionBits);
    ledcAttachPin(_verticalServoPin, kVerticalPwmChannel);

    // Estado inicial
    _currentAngle         = _maxAngle;       // repouso em 180° (extremidade)
    _verticalCurrentAngle = _verticalRestAngle;
    _fireAngle            = 0;
    _fireDetected         = false;
    _hasNewData           = false;
    _sweepIncreasing      = false;           // primeira varredura: 180° → minAngle
    _lastStepMillis       = 0;
    _candidateStartedAt   = 0;
    _fireDirection        = FireDirection::None;
    _detectionEnabled     = false;
    _sweepEnabled         = false;

    // Posição inicial: horizontal 180°, vertical em repouso
    writeServoAngle(_currentAngle);
    writeVerticalAngle(_verticalCurrentAngle);

    Serial.printf("FIRE SCANNER READY: H=%u V=%u threshold=%u activeLow=%s\n",
                  _currentAngle, _verticalCurrentAngle,
                  _fireThreshold, _activeWhenLow ? "true" : "false");
}

// ── Loop de atualização ───────────────────────────────────────────────────────

void FireScanner::update(unsigned long currentMillis) {
    if (_sweepEnabled) {
        updateSweep(currentMillis);
    }
    updateFireReading(currentMillis);
}

// ── Varredura horizontal ──────────────────────────────────────────────────────

void FireScanner::updateSweep(unsigned long currentMillis) {
    if (currentMillis - _lastStepMillis < _stepIntervalMillis) return;
    _lastStepMillis = currentMillis;

    if (_sweepIncreasing) {
        if (_currentAngle + _stepDegrees >= _maxAngle) {
            _currentAngle    = _maxAngle;
            _sweepIncreasing = false;
        } else {
            _currentAngle += _stepDegrees;
        }
    } else {
        if (_currentAngle <= _minAngle + _stepDegrees) {
            _currentAngle    = _minAngle;
            _sweepIncreasing = true;
        } else {
            _currentAngle -= _stepDegrees;
        }
    }
    writeServoAngle(_currentAngle);
}

// ── Detecção de chama — sensor E27-004 ───────────────────────────────────────
//
// Estratégia: comparação direta de threshold, sem baseline.
//
// Sensor E27-004 (comportamento observado, _activeWhenLow = false):
//   - Sem chama → ADC natural baixo (~110)
//   - Com chama → sinal sobe (fotodiodo gera corrente, A0 aumenta)
//   - Detecção: raw >= _fireThreshold  (padrão 500)
//
// Se o módulo funcionar ao contrário (setSensorActiveWhenLow(true)):
//   - Detecção: raw <= _fireThreshold
//
// TUNING — use o campo "flameRaw" da telemetria WebSocket:
//   1. Sem chama: anote flameRaw em repouso        → ex.: 110
//   2. Com isqueiro: anote flameRaw com chama      → ex.: 1200
//   3. threshold ideal = ponto médio               → ex.: (110+1200)/2 ≈ 650
//   4. Ajuste com setFireThreshold() ou no header.

void FireScanner::updateFireReading(unsigned long currentMillis) {
    // Lê o sensor sempre — necessário para telemetria independente do estado de detecção.
    bool sampleFire = false;
    if (_digitalMode) {
        // Modo digital (D0): LOW = chama detectada (ativo-baixo com pull-up interno)
        const int d = digitalRead(_flameSensorPin);
        _lastRaw  = (d == LOW) ? 1 : 0; // 1 = chama, 0 = sem chama (para telemetria)
        sampleFire = (d == LOW);
    } else {
        // Modo analógico (A0): compara com threshold
        const int rawValue = analogRead(_flameSensorPin);
        if (rawValue >= 0) _lastRaw = static_cast<uint16_t>(rawValue);
        const uint16_t raw = _lastRaw;
        sampleFire = _activeWhenLow ? (raw <= _fireThreshold) : (raw >= _fireThreshold);
    }

    if (!_detectionEnabled) {
        if (_fireDetected) _hasNewData = true;
        _fireDetected       = false;
        _fireDirection      = FireDirection::None;
        _candidateStartedAt = 0;
        return;
    }

    // Impressão diagnóstica — pode comentar após calibração
    static unsigned long lastPrintMs = 0;
    if (currentMillis - lastPrintMs >= 250) {
        lastPrintMs = currentMillis;
        if (_digitalMode) {
            Serial.printf("FLAME: digital=%s  angle=%3u  detected=%s\n",
                          sampleFire ? "FIRE(LOW)" : "none(HIGH)",
                          _currentAngle,
                          _fireDetected ? "YES" : "no");
        } else {
            Serial.printf("FLAME: raw=%4u  threshold=%u  mode=%s  angle=%3u  detected=%s\n",
                          _lastRaw, _fireThreshold,
                          _activeWhenLow ? "raw<=thr" : "raw>=thr",
                          _currentAngle,
                          _fireDetected ? "YES" : "no");
        }
    }

    // Confirmação por tempo — filtra ruído elétrico e reflexos
    if (sampleFire) {
        if (_candidateStartedAt == 0) {
            _candidateStartedAt = currentMillis;
        }
    } else {
        _candidateStartedAt = 0;
    }

    const bool detected = (_candidateStartedAt != 0) &&
                          (currentMillis - _candidateStartedAt >= _confirmationDurationMillis);

    const FireDirection prevDirection = _fireDirection;

    if (detected) {
        _fireAngle     = _currentAngle;
        _fireDirection = directionForAngle(_fireAngle);
    } else {
        _fireDirection = FireDirection::None;
    }

    if (detected != _fireDetected || _fireDirection != prevDirection) {
        _hasNewData = true;
    }

    _fireDetected = detected;
}

// ── Controle de estado ────────────────────────────────────────────────────────

void FireScanner::enableDetection(bool enabled, unsigned long currentMillis) {
    if (enabled == _detectionEnabled) return;

    if (!enabled) {
        resetDetection();
    }

    _detectionEnabled = enabled;
    Serial.printf("FIRE DETECTION: %s\n", enabled ? "ENABLED" : "DISABLED");
}

void FireScanner::enableSweep(bool enabled) {
    if (enabled == _sweepEnabled) return;
    _sweepEnabled = enabled;

    if (enabled) {
        // Sobe o servo vertical para a posição de busca
        _verticalCurrentAngle = _verticalSearchAngle;
        writeVerticalAngle(_verticalCurrentAngle);
        // Inicia varredura horizontal a partir de 180° (maxAngle → minAngle)
        _currentAngle    = _maxAngle;
        _sweepIncreasing = false;
        writeServoAngle(_currentAngle);
        Serial.printf("FIRE SWEEP ENABLED: V=%u H_sweep=%u..%u (inicio em %u)\n",
                      _verticalSearchAngle, _minAngle, _maxAngle, _maxAngle);
    } else {
        Serial.println("FIRE SWEEP DISABLED");
    }
}

/// Retorna horizontal a 180° (maxAngle) e vertical ao ângulo de repouso.
void FireScanner::center() {
    _currentAngle         = _maxAngle;       // repouso em 180°
    _verticalCurrentAngle = _verticalRestAngle;
    _fireAngle            = 0;
    _fireDirection        = FireDirection::None;
    _fireDetected         = false;
    _candidateStartedAt   = 0;
    _sweepIncreasing      = false;           // próxima varredura começa descendo de 180°

    writeServoAngle(_currentAngle);
    writeVerticalAngle(_verticalCurrentAngle);

    Serial.printf("FIRE SCANNER CENTERED: H=%u V=%u\n", _maxAngle, _verticalRestAngle);
}

bool FireScanner::sweepEnabled() const { return _sweepEnabled; }

// ── Accessors ─────────────────────────────────────────────────────────────────

bool          FireScanner::hasNewData()     const { return _hasNewData; }
void          FireScanner::clearNewData()         { _hasNewData = false; }
bool          FireScanner::fireDetected()   const { return _fireDetected; }
FireDirection FireScanner::fireDirection()  const { return _fireDirection; }
uint8_t       FireScanner::fireAngle()      const { return _fireAngle; }
uint8_t       FireScanner::currentAngle()   const { return _currentAngle; }
uint16_t      FireScanner::rawValue()       const { return _lastRaw; }

// ── Configuração ──────────────────────────────────────────────────────────────

void FireScanner::setSweepRange(uint8_t minAngle, uint8_t maxAngle) {
    if (minAngle >= maxAngle || maxAngle > 180) return;
    _minAngle     = minAngle;
    _maxAngle     = maxAngle;
    _currentAngle = clampAngle(_currentAngle);
    writeServoAngle(_currentAngle);
}

void FireScanner::setDirectionCenterAngle(uint8_t centerAngle) {
    if (centerAngle <= 180) _directionCenterAngle = centerAngle;
}

void FireScanner::setStepInterval(unsigned long intervalMillis) {
    _stepIntervalMillis = intervalMillis;
}

void FireScanner::setStepDegrees(uint8_t stepDegrees) {
    if (stepDegrees > 0) _stepDegrees = stepDegrees;
}

void FireScanner::setVerticalSearchAngle(uint8_t angle) {
    if (angle <= 180) _verticalSearchAngle = angle;
}

void FireScanner::setVerticalRestAngle(uint8_t angle) {
    if (angle <= 180) _verticalRestAngle = angle;
}

void FireScanner::setVerticalServoPulseRange(uint16_t minPulseMicros, uint16_t maxPulseMicros) {
    if (minPulseMicros < maxPulseMicros) {
        _minPulseMicrosVertical = minPulseMicros;
        _maxPulseMicrosVertical = maxPulseMicros;
        writeVerticalAngle(_verticalCurrentAngle);
    }
}

void FireScanner::setFireThreshold(uint16_t threshold) {
    if (threshold <= 4095) _fireThreshold = threshold;
}

void FireScanner::setSensorActiveWhenLow(bool activeWhenLow) {
    _activeWhenLow = activeWhenLow;
}

void FireScanner::setDigitalMode(bool useDigital) {
    _digitalMode = useDigital;
}

void FireScanner::setConfirmationDuration(unsigned long ms) {
    _confirmationDurationMillis = ms;
}

void FireScanner::setServoPulseRange(uint16_t minPulseMicros, uint16_t maxPulseMicros) {
    if (minPulseMicros < maxPulseMicros) {
        _minPulseMicros = minPulseMicros;
        _maxPulseMicros = maxPulseMicros;
        writeServoAngle(_currentAngle);
    }
}

// ── Estado interno ────────────────────────────────────────────────────────────

void FireScanner::resetDetection() {
    _fireDetected       = false;
    _hasNewData         = false;
    _fireDirection      = FireDirection::None;
    _candidateStartedAt = 0;
}

// ── Controle dos servos ───────────────────────────────────────────────────────

void FireScanner::writeServoAngle(uint8_t angle) {
    const uint16_t pulse = pulseForAngle(angle, _minPulseMicros, _maxPulseMicros);
    const uint32_t duty  = (static_cast<uint32_t>(pulse) * kMaxDuty) / kServoPeriodMicros;
    ledcWrite(kHorizontalPwmChannel, duty);
}

void FireScanner::writeVerticalAngle(uint8_t angle) {
    const uint16_t pulse = pulseForAngle(angle, _minPulseMicrosVertical, _maxPulseMicrosVertical);
    const uint32_t duty  = (static_cast<uint32_t>(pulse) * kMaxDuty) / kServoPeriodMicros;
    ledcWrite(kVerticalPwmChannel, duty);
}

uint16_t FireScanner::pulseForAngle(uint8_t angle,
                                    uint16_t minPulse,
                                    uint16_t maxPulse) const {
    const uint32_t range = maxPulse - minPulse;
    return static_cast<uint16_t>(minPulse + (range * angle) / 180);
}

FireDirection FireScanner::directionForAngle(uint8_t angle) const {
    static const uint8_t kWindow = 20;
    if (angle + kWindow < _directionCenterAngle) return FireDirection::Left;
    if (angle > _directionCenterAngle + kWindow)  return FireDirection::Right;
    return FireDirection::Center;
}

uint8_t FireScanner::clampAngle(uint8_t angle) const {
    if (angle < _minAngle) return _minAngle;
    if (angle > _maxAngle) return _maxAngle;
    return angle;
}

} // namespace sensors
