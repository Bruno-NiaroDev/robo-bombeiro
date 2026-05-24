#pragma once

#include <stdint.h>

namespace drivers {

class MotorDriver {
public:
    enum class Direction {
        Stopped,
        Forward,
        Backward,
        TurnLeft,
        TurnRight
    };

    void begin();
    void update(unsigned long currentMillis);

    void forward(uint8_t speed);
    void backward(uint8_t speed);
    void turnLeft(uint8_t speed);
    void turnRight(uint8_t speed);
    void stop();

    // Correção diferencial durante avanço (correction > 0 → vira à esquerda)
    void driveWithCorrection(uint8_t speed, int8_t correction);
    // Correção diferencial durante recuo — sinal invertido em relação ao avanço
    void driveBackwardWithCorrection(uint8_t speed, int8_t correction);

    uint8_t speed() const;
    Direction direction() const;

private:
    void applyState();
    void setMotorPower(int16_t leftPower, int16_t rightPower);

    Direction _direction = Direction::Stopped;
    uint8_t _speed = 0;
    bool _dirty = false;
};

} // namespace drivers
