#include <Arduino.h>
#include "robot/AutonomousRobotSystem.h"

static robot::AutonomousRobotSystem robotSystem;

void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println("ROBO BOMBEIRO DEBUG START");
    robotSystem.begin();
}

void loop() {
    robotSystem.update(millis());
}

