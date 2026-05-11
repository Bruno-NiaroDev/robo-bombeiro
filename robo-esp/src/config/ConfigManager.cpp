#include "config/ConfigManager.h"
#include "config/NetworkCredentials.h"

namespace config {

ConfigManager& ConfigManager::instance() {
    static ConfigManager configManager;
    return configManager;
}

void ConfigManager::begin() {
    _networkConfig.wifiSsid = ROBO_WIFI_SSID;
    _networkConfig.wifiPassword = ROBO_WIFI_PASSWORD;
    _networkConfig.wsHost = ROBO_WS_HOST;
    _networkConfig.wsPort = ROBO_WS_PORT;
    _networkConfig.wsPath = ROBO_WS_PATH;
}

void ConfigManager::update(unsigned long currentMillis) {
    (void)currentMillis;
}

const RobotConfig& ConfigManager::robotConfig() const {
    return _robotConfig;
}

const NetworkConfig& ConfigManager::networkConfig() const {
    return _networkConfig;
}

const NavigationConfig& ConfigManager::navigationConfig() const {
    return _navigationConfig;
}

} // namespace config
