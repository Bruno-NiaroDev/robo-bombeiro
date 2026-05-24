#include "drivers/ImuDriver.h"
#include "Pins.h"
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

namespace drivers {

// ── Registradores do MPU6050 ────────────────────────────────────────────────
static const uint8_t kRegPwrMgmt1  = 0x6B;
static const uint8_t kRegConfig     = 0x1A;
static const uint8_t kRegSmplrtDiv  = 0x19;
static const uint8_t kRegGyroConfig = 0x1B;
static const uint8_t kRegGyroZOutH  = 0x47;
static const uint8_t kRegWhoAmI     = 0x75;

// Número de amostras para calibração (300 × 3 ms ≈ 0,9 s de aquisição)
static const int kCalibrationSamples = 300;
// Fator de rejeição de outliers (amostras a mais de kSigmaReject σ são descartadas)
static constexpr float kSigmaReject = 2.0f;
// Taxa máxima de atualização: 1 leitura a cada 2 ms → 500 Hz máximo
static const unsigned long kMinUpdateIntervalMicros = 2000;

// ── Inicialização ────────────────────────────────────────────────────────────

bool ImuDriver::begin(int sdaPin, int sclPin) {
    Wire.begin(sdaPin, sclPin);
    Wire.setClock(400000);  // I2C Fast Mode 400 kHz

    delay(50);  // aguarda estabilização do chip

    // Verifica WHO_AM_I (deve ser 0x68 no MPU-6050 ou 0x72 em clones)
    Wire.beginTransmission(kAddress);
    Wire.write(kRegWhoAmI);
    if (Wire.endTransmission(false) != 0) {
        Serial.println("IMU ERROR: sem resposta em 0x68");
        return false;
    }
    Wire.requestFrom((uint8_t)kAddress, (uint8_t)1);
    if (!Wire.available()) {
        Serial.println("IMU ERROR: sem dados WHO_AM_I");
        return false;
    }
    uint8_t whoAmI = Wire.read();
    if (whoAmI != 0x68 && whoAmI != 0x72) {
        Serial.printf("IMU ERROR: WHO_AM_I inesperado=0x%02X\n", whoAmI);
        return false;
    }

    // Usa PLL do giroscópio X como referência de clock (mais estável que oscilador interno)
    writeReg(kRegPwrMgmt1, 0x01);
    delay(100);

    // DLPF = 44 Hz: suaviza ruído de alta frequência sem introduzir lag perceptível
    // para movimentos robóticos (gyro BW ≈ 44 Hz, delay ≈ 4,8 ms)
    writeReg(kRegConfig, 0x03);

    // Taxa de amostragem = gyroOutputRate / (SMPLRT_DIV + 1)
    // Com DLPF ativo, gyroOutputRate = 1000 Hz → 1000/(9+1) = 100 Hz
    writeReg(kRegSmplrtDiv, 0x09);

    // Escala ±500°/s: sensibilidade 65,5 LSB/(°/s)
    // Velocidade de curva típica ≤ 400°/s → escala ±500 nunca satura
    writeReg(kRegGyroConfig, 0x08);

    delay(50);

    calibrate();

    _yaw       = 0.0f;
    _yawRate   = 0.0f;
    _lastMicros = micros();
    _ready     = true;
    Serial.printf("IMU PRONTO: bias=%.4f deg/s alpha=%.2f\n", _biasDegsPerSec, _filterAlpha);
    return true;
}

// ── Calibração com rejeição de outliers ─────────────────────────────────────
//
// Algoritmo de duas passagens:
//   1ª passagem: coleta amostras e calcula média e desvio padrão
//   2ª passagem: descarta amostras a mais de kSigmaReject σ da média
//                e recalcula a média final (bias)
//
// Isso elimina leituras espúrias causadas por vibração ou toque acidental
// no robô durante a calibração.

void ImuDriver::calibrate() {
    Serial.println("IMU CALIBRANDO: mantenha o robo parado...");

    float readings[kCalibrationSamples];
    int   valid = 0;

    for (int i = 0; i < kCalibrationSamples; ++i) {
        int16_t raw = 0;
        if (readRawGyroZ(raw)) {
            readings[valid++] = raw * kSensitivity;
        }
        delay(3);  // 3 ms → ~333 Hz coleta (oversampling intencional)
    }

    if (valid < 10) {
        _biasDegsPerSec = 0.0f;
        Serial.println("IMU CALIBRACAO FALHOU: amostras insuficientes");
        return;
    }

    // 1ª passagem: média e desvio padrão
    float sum = 0.0f;
    for (int i = 0; i < valid; ++i) sum += readings[i];
    const float mean = sum / valid;

    float varSum = 0.0f;
    for (int i = 0; i < valid; ++i) {
        const float d = readings[i] - mean;
        varSum += d * d;
    }
    const float stddev = sqrtf(varSum / valid);

    // 2ª passagem: rejeita outliers e recalcula média
    float filteredSum   = 0.0f;
    int   filteredCount = 0;
    int   outliers      = 0;
    for (int i = 0; i < valid; ++i) {
        if (fabsf(readings[i] - mean) <= kSigmaReject * stddev) {
            filteredSum += readings[i];
            ++filteredCount;
        } else {
            ++outliers;
        }
    }

    _biasDegsPerSec = (filteredCount > 0) ? (filteredSum / filteredCount) : mean;

    Serial.printf(
        "IMU CALIBRACAO OK: amostras=%d bias=%.5f deg/s stddev=%.5f outliers=%d\n",
        filteredCount, _biasDegsPerSec, stddev, outliers
    );
}

// ── Loop de atualização ──────────────────────────────────────────────────────
//
// Deve ser chamado o mais rápido possível (preferencialmente em cada iteração
// do loop principal).  A lógica interna limita a taxa de integração em 500 Hz.
//
// Filtro EMA (Exponential Moving Average):
//   yawRate_filtrado = α × rawRate + (1 − α) × yawRate_anterior
//
// Reduz ruído de alta frequência sem introduzir delay de fase perceptível
// para rotações robóticas (geralmente abaixo de 400°/s).

void ImuDriver::update() {
    if (!_ready) return;

    unsigned long now = micros();
    unsigned long dtMicros = now - _lastMicros;
    if (dtMicros < kMinUpdateIntervalMicros) return;
    _lastMicros = now;

    int16_t rawZ = 0;
    if (!readRawGyroZ(rawZ)) return;

    // Converte, remove bias e aplica inversão se necessário
    float rawRate = rawZ * kSensitivity - _biasDegsPerSec;
    if (_inverted) rawRate = -rawRate;

    // Filtro EMA na taxa de rotação
    _yawRate = _filterAlpha * rawRate + (1.0f - _filterAlpha) * _yawRate;

    // Integra yaw usando a taxa filtrada
    _yaw += _yawRate * (dtMicros * 1e-6f);
}

// ── Accessors ────────────────────────────────────────────────────────────────

float ImuDriver::yaw()     const { return _yaw; }
float ImuDriver::yawRate() const { return _yawRate; }
bool  ImuDriver::isReady() const { return _ready; }

void ImuDriver::resetYaw() {
    _yaw = 0.0f;
}

void ImuDriver::setYawInverted(bool inverted) {
    _inverted = inverted;
}

void ImuDriver::setFilterAlpha(float alpha) {
    _filterAlpha = alpha < 0.0f ? 0.0f : (alpha > 1.0f ? 1.0f : alpha);
}

// ── Comunicação I2C ──────────────────────────────────────────────────────────

bool ImuDriver::readRawGyroZ(int16_t& rawZ) {
    Wire.beginTransmission(kAddress);
    Wire.write(kRegGyroZOutH);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom((uint8_t)kAddress, (uint8_t)2);
    if (Wire.available() < 2) return false;
    const uint8_t hi = Wire.read();
    const uint8_t lo = Wire.read();
    rawZ = static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | lo);
    return true;
}

void ImuDriver::writeReg(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(kAddress);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

} // namespace drivers
