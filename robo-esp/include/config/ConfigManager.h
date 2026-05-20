#pragma once

#include <stdint.h>

namespace config {

struct RobotConfig {
    uint8_t gridWidth = 4;   // deve coincidir com GridMap::Width
    uint8_t gridHeight = 4;  // deve coincidir com GridMap::Height
    uint8_t startX = 0;      // deve coincidir com GridMap::Home
    uint8_t startY = 0;
};

struct NetworkConfig {
    const char* wifiSsid = "";
    const char* wifiPassword = "";
    const char* wsHost = "192.168.0.10";
    uint16_t wsPort = 55619;
    const char* wsPath = "/?channelCode=fire1";
    bool enableWebSocket = true;
    unsigned long wifiReconnectIntervalMs = 5000;
    unsigned long telemetryIntervalMs = 1000;
};

struct NavigationConfig {
    uint16_t updateIntervalMs = 50;
    uint16_t cellTravelDurationMs = 950;
    uint16_t turn90DurationMs = 600;
    uint16_t movementPauseDurationMs = 200;
    uint8_t moveSpeed = 140;
    uint8_t turnSpeed = 140;
};

class ConfigManager {
public:
    static ConfigManager& instance();
    void begin();
    void update(unsigned long currentMillis);

    const RobotConfig& robotConfig() const;
    const NetworkConfig& networkConfig() const;
    const NavigationConfig& navigationConfig() const;

private:
    ConfigManager() = default;
    RobotConfig _robotConfig;
    NetworkConfig _networkConfig;
    NavigationConfig _navigationConfig;
};

} // namespace config
