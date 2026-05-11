#pragma once

#include "WebSocketClient.h"

namespace network {

class NetworkManager {
public:
    void begin();
    void update(unsigned long currentMillis);
    void broadcastState(const char* payload);
    bool isConnected() const;

    bool hasTargetPosition() const;
    TargetPositionMessage consumeTargetPosition();
    bool sendTelemetry(const TelemetryPayload& telemetry);

private:
    WebSocketClient _webSocketClient;
};

} // namespace network
