#pragma once

// Preencha estes valores antes de gravar no ESP32.
// O host deve ser o IP da maquina que esta rodando web-socket-server/server.js
// na mesma rede Wi-Fi do ESP32, nao "localhost".

#define ROBO_WIFI_SSID "SUA_REDE_WIFI"
#define ROBO_WIFI_PASSWORD "SUA_SENHA_WIFI"

#define ROBO_WS_HOST "SUA_HOSPEDEIRA"
#define ROBO_WS_PORT 55619
#define ROBO_WS_PATH "/?channelCode=fire1"