#include "network/NetworkManager.h"
#include "config/ConfigManager.h"

namespace network {

void NetworkManager::begin() {
    _webSocketClient.begin(config::ConfigManager::instance().networkConfig());
}

void NetworkManager::update(unsigned long currentMillis) {
    _webSocketClient.update(currentMillis);
}

void NetworkManager::broadcastState(const char* payload) {
    _webSocketClient.sendJson(payload);
}

bool NetworkManager::isConnected() const {
    return _webSocketClient.isConnected();
}

bool NetworkManager::hasTargetPosition() const {
    return _webSocketClient.hasTargetPosition();
}

TargetPositionMessage NetworkManager::consumeTargetPosition() {
    return _webSocketClient.consumeTargetPosition();
}

bool NetworkManager::sendTelemetry(const TelemetryPayload& telemetry) {
    return _webSocketClient.sendTelemetry(telemetry);
}

} // namespace network
