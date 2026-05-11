#include "network/WebSocketClient.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <string.h>

namespace network {

void WebSocketClient::begin(const config::NetworkConfig& config) {
    _config = &config;
    _targetPosition = TargetPositionMessage();
    _lastWifiAttemptMillis = 0;
    _webSocketStarted = false;
    _webSocketConnected = false;

    WiFi.mode(WIFI_STA);

    if (_config->enableWebSocket) {
        connectWifi(0);
    }
}

void WebSocketClient::update(unsigned long currentMillis) {
    if (!_config || !_config->enableWebSocket) {
        return;
    }

    if (!isWifiConnected()) {
        _webSocketConnected = false;
        connectWifi(currentMillis);
        return;
    }

    if (!_webSocketStarted) {
        connectWebSocket();
    }

    _webSocket.loop();
}

bool WebSocketClient::isWifiConnected() const {
    return WiFi.status() == WL_CONNECTED;
}

bool WebSocketClient::isWebSocketConnected() const {
    return _webSocketConnected;
}

bool WebSocketClient::isConnected() const {
    return isWifiConnected() && isWebSocketConnected();
}

bool WebSocketClient::hasTargetPosition() const {
    return _targetPosition.available;
}

TargetPositionMessage WebSocketClient::consumeTargetPosition() {
    TargetPositionMessage message = _targetPosition;
    _targetPosition.available = false;
    return message;
}

bool WebSocketClient::sendTelemetry(const TelemetryPayload& telemetry) {
    StaticJsonDocument<384> doc;
    doc["type"] = "TELEMETRY";

    JsonObject payload = doc.createNestedObject("payload");
    payload["x"] = telemetry.x;
    payload["y"] = telemetry.y;
    payload["state"] = telemetry.state;
    payload["pathSize"] = telemetry.pathSize;
    payload["targetX"] = telemetry.targetX;
    payload["targetY"] = telemetry.targetY;
    payload["obstacleDetected"] = telemetry.obstacleDetected;
    payload["fireDetected"] = telemetry.fireDetected;
    payload["pumpOn"] = telemetry.pumpOn;
    payload["lastError"] = telemetry.lastError;

    char buffer[384];
    size_t length = serializeJson(doc, buffer, sizeof(buffer));

    if (length == 0 || length >= sizeof(buffer)) {
        Serial.println("TELEMETRY ERROR: JSON buffer too small or serialization failed");
        return false;
    }

    return sendJson(buffer);
}

bool WebSocketClient::sendJson(const char* json) {
    if (!json || !isConnected()) {
        return false;
    }

    _webSocket.sendTXT(json);
    return true;
}

void WebSocketClient::connectWifi(unsigned long currentMillis) {
    if (!_config || !_config->wifiSsid || _config->wifiSsid[0] == '\0') {
        Serial.println("WIFI ERROR: SSID not configured");
        return;
    }

    if (currentMillis != 0 &&
        currentMillis - _lastWifiAttemptMillis < _config->wifiReconnectIntervalMs) {
        return;
    }

    _lastWifiAttemptMillis = currentMillis;
    Serial.print("WIFI CONNECTING: ");
    Serial.println(_config->wifiSsid);
    WiFi.disconnect(false);
    WiFi.begin(_config->wifiSsid, _config->wifiPassword);
}

void WebSocketClient::connectWebSocket() {
    if (!_config || !_config->wsHost || _config->wsHost[0] == '\0') {
        Serial.println("WEBSOCKET ERROR: host not configured");
        return;
    }

    Serial.printf("WEBSOCKET CONNECTING: %s:%u%s\n", _config->wsHost, _config->wsPort, _config->wsPath);
    _webSocket.begin(_config->wsHost, _config->wsPort, _config->wsPath);
    _webSocket.onEvent([this](WStype_t type, uint8_t* payload, size_t length) {
        handleEvent(type, payload, length);
    });
    _webSocket.setReconnectInterval(5000);
    _webSocketStarted = true;
}

void WebSocketClient::handleEvent(WStype_t type, uint8_t* payload, size_t length) {
    switch (type) {
        case WStype_CONNECTED:
            _webSocketConnected = true;
            Serial.println("WEBSOCKET CONNECTED");
            break;
        case WStype_DISCONNECTED:
            _webSocketConnected = false;
            Serial.println("WEBSOCKET DISCONNECTED");
            break;
        case WStype_TEXT:
            handleTextMessage(payload, length);
            break;
        default:
            break;
    }
}

void WebSocketClient::handleTextMessage(uint8_t* payload, size_t length) {
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, payload, length);

    if (error) {
        Serial.print("WEBSOCKET JSON ERROR: ");
        Serial.println(error.c_str());
        return;
    }

    const char* type = doc["type"] | "";
    Serial.print("WEBSOCKET MESSAGE TYPE: ");
    Serial.println(type);

    if (strcmp(type, "TARGET_POSITION") == 0) {
        handleTargetPosition(doc["payload"]);
    } else {
        Serial.print("WEBSOCKET MESSAGE IGNORED: unknown type ");
        Serial.println(type);
    }
}

void WebSocketClient::handleTargetPosition(JsonVariantConst payload) {
    if (!payload.is<JsonObjectConst>()) {
        Serial.println("TARGET_POSITION ERROR: payload is not an object");
        return;
    }

    if (!payload["x"].is<uint8_t>() || !payload["y"].is<uint8_t>()) {
        Serial.println("TARGET_POSITION ERROR: x/y missing or outside uint8 range");
        return;
    }

    _targetPosition.x = payload["x"].as<uint8_t>();
    _targetPosition.y = payload["y"].as<uint8_t>();
    _targetPosition.available = true;
    Serial.printf("TARGET RECEIVED: (%u,%u)\n", _targetPosition.x, _targetPosition.y);
}

} // namespace network
