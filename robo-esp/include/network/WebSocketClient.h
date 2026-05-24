#pragma once

#include <stdint.h>
#include <stddef.h>
#include <ArduinoJson.h>
#include <WebSocketsClient.h>
#include "config/ConfigManager.h"

namespace network {

enum class NetworkMessageType {
    None,
    TargetPosition,
    Unknown
};

struct TargetPositionMessage {
    uint8_t x = 0;
    uint8_t y = 0;
    bool available = false;
};

struct TelemetryPayload {
    uint8_t x = 0;
    uint8_t y = 0;
    uint8_t heading = 2;
    bool imuReady = false;
    float imuYaw = 0.0f;
    const char* state = "IDLE";
    uint8_t pathSize = 0;
    uint8_t targetX = 0;
    uint8_t targetY = 0;
    bool obstacleDetected = false;
    bool fireDetected = false;
    bool pumpOn = false;
    uint16_t flameRaw = 0;   ///< Leitura ADC bruta do sensor de chama (0–4095). Para calibração.
    const char* lastError = "";
};

class WebSocketClient {
public:
    void begin(const config::NetworkConfig& config);
    void update(unsigned long currentMillis);

    bool isWifiConnected() const;
    bool isWebSocketConnected() const;
    bool isConnected() const;

    bool hasTargetPosition() const;
    TargetPositionMessage consumeTargetPosition();

    bool sendTelemetry(const TelemetryPayload& telemetry);
    bool sendJson(const char* json);

private:
    void connectWifi(unsigned long currentMillis);
    void connectWebSocket();
    void handleEvent(WStype_t type, uint8_t* payload, size_t length);
    void handleTextMessage(uint8_t* payload, size_t length);
    void handleTargetPosition(JsonVariantConst payload);

    const config::NetworkConfig* _config = nullptr;
    WebSocketsClient _webSocket;
    TargetPositionMessage _targetPosition;
    unsigned long _lastWifiAttemptMillis = 0;
    bool _webSocketStarted = false;
    bool _webSocketConnected = false;
};

} // namespace network
