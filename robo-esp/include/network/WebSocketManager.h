#pragma once

namespace network {

class WebSocketManager {
public:
    void begin(uint16_t port);
    void sendText(const char* message);
    void update(unsigned long currentMillis);
    bool isClientConnected() const;
};

} // namespace network
