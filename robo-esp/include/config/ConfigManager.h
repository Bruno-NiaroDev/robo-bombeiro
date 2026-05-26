#pragma once

#include <stdint.h>

namespace config {

struct RobotConfig {
    uint8_t gridWidth  = 4;   // deve coincidir com GridMap::Width
    uint8_t gridHeight = 4;   // deve coincidir com GridMap::Height
    uint8_t startX     = 0;   // deve coincidir com GridMap::Home.x
    uint8_t startY     = 0;   // deve coincidir com GridMap::Home.y
};

struct NetworkConfig {
    const char*   wifiSsid                = "";
    const char*   wifiPassword            = "";
    const char*   wsHost                  = "192.168.0.10";
    uint16_t      wsPort                  = 55619;
    const char*   wsPath                  = "/?channelCode=fire1";
    bool          enableWebSocket         = true;
    unsigned long wifiReconnectIntervalMs = 5000;
    unsigned long telemetryIntervalMs     = 1000;
};

struct NavigationConfig {
    // ── Temporização de movimento ────────────────────────────────────────────
    uint16_t cellTravelDurationMs    = 950;   // duração de cada célula (ms)
    uint16_t turn90DurationMs        = 700;   // timeout de segurança da curva (ms)
    uint16_t movementPauseDurationMs = 280;   // pausa entre giro e avanço (ms)
    uint16_t updateIntervalMs        = 50;    // intervalo de update (ms)

    // ── Velocidades ──────────────────────────────────────────────────────────
    uint8_t moveSpeed  = 140;   // PWM de avanço reto [0–255]
    uint8_t turnSpeed  = 140;   // PWM máximo de curva [0–255]
    uint8_t minTurnSpeed = 55;  // PWM mínimo durante curva (zona morta do motor)
                                // ↑ aumente se o robô travar perto dos 90°
                                // ↓ reduza se overshoot for grande

    // ── Orientação inicial ───────────────────────────────────────────────────
    // Orientação física do robô em relação ao grid ao ligar:
    //   0 = NORTH   1 = EAST   2 = SOUTH (padrão)   3 = WEST
    // Troque para 1 (EAST) se o robô virar na primeira coordenada recebida.
    uint8_t initialOrientationValue = 2;

    // ── IMU ──────────────────────────────────────────────────────────────────
    // true se o robô deriva ao contrário do esperado (chip montado de cabeça pra baixo)
    bool  imuYawInverted = false;
    // true se curva à esquerda aumenta yaw integrado (padrão). false se motores invertidos.
    bool  turnLeftPositiveYaw = true;
    // Coeficiente do filtro EMA da taxa de rotação do giroscópio:
    //   1.0 = sem filtro   0.7 = padrão recomendado   0.4 = mais suave (+ lag)
    float imuFilterAlpha = 0.7f;

    // ── Tolerância de curva ──────────────────────────────────────────────────
    // Graus antes de 90° em que o motor é cortado; inércia completa o restante.
    //   Reduza (p. ex. 1.0°) se o robô para antes dos 90°.
    //   Aumente (p. ex. 3.0–4.0) se ultrapassar os 90° (overshoot).
    float turnToleranceDeg = 3.0f;

    // ── PID de curva ─────────────────────────────────────────────────────────
    // Controla a velocidade durante rotações com base no ângulo restante.
    // Saída: velocidade PWM em [minTurnSpeed, turnSpeed].
    //
    //   kp_turn: ganho principal. Aumente para curvas mais rápidas no início;
    //            valores altos podem causar overshoot nos últimos graus.
    //   ki_turn: ajuda a vencer atrito estático nos últimos graus.
    //            Mantenha pequeno (0.1–0.4) para evitar windup.
    //   kd_turn: amorte-ce a aproximação ao alvo. Aumente se overshoot persistir.
    float turnPidKp = 2.5f;
    float turnPidKi = 0.2f;
    float turnPidKd = 0.5f;

    // ── PID de heading ───────────────────────────────────────────────────────
    // Corrige deriva lateral durante avanço e recuo reto.
    // Saída: correção diferencial PWM em [-25, +25].
    //
    //   kp_heading: ganho principal. O valor 1.8 produz ≈ ±5 PWM a 3° de erro.
    //   ki_heading: corrige deriva sistemática (bateria fraca, superfície inclinada).
    //   kd_heading: amorte-ce oscilações em superfícies escorregadias.
    float headingPidKp = 1.8f;
    float headingPidKi = 0.05f;
    float headingPidKd = 0.02f;
};

class ConfigManager {
public:
    static ConfigManager& instance();
    void begin();
    void update(unsigned long currentMillis);

    const RobotConfig&      robotConfig()      const;
    const NetworkConfig&    networkConfig()     const;
    const NavigationConfig& navigationConfig()  const;

private:
    ConfigManager() = default;
    RobotConfig      _robotConfig;
    NetworkConfig    _networkConfig;
    NavigationConfig _navigationConfig;
};

} // namespace config
