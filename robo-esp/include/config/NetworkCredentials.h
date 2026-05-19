#pragma once

// Preencha estes valores antes de gravar no ESP32.
// O host deve ser o IP da maquina que esta rodando web-socket-server/server.js
// na mesma rede Wi-Fi do ESP32, nao "localhost".

#define ROBO_WIFI_SSID "Bruno"
#define ROBO_WIFI_PASSWORD "36338081b"

#define ROBO_WS_HOST "niarodev.com.br"
#define ROBO_WS_PORT 55619
#define ROBO_WS_PATH "/?channelCode=fire1"