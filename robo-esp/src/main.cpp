#include <Arduino.h>
#include "robot/AutonomousRobotSystem.h"
#include "Pins.h"

static robot::AutonomousRobotSystem robotSystem;

void setup() {
    // ── Segurança: desativa o relé da bomba ANTES de qualquer inicialização ──────
    // O ESP32 mantém pinos em alta-impedância durante o boot (~300ms).
    // Relé ativo-baixo vê esse estado flutuante como LOW e ativa a bomba.
    // Configurar OUTPUT + HIGH aqui garante relay desligado desde o início.
    pinMode(pins::pump::RELAY, OUTPUT);
    digitalWrite(pins::pump::RELAY, HIGH); // HIGH = relay off (ativo-baixo)

    Serial.begin(115200);
    delay(300);
    Serial.println("ROBO BOMBEIRO DEBUG START");
    robotSystem.begin();
}

void loop() {
    robotSystem.update(millis());
}

