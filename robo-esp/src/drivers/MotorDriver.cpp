#include "drivers/MotorDriver.h"
#include "Pins.h"
#include <Arduino.h>

namespace drivers {

static const int kLeftChannel = 0;
static const int kRightChannel = 1;
static const int kPwmResolution = 8;
static const int kMaxDuty = (1 << kPwmResolution) - 1;

void MotorDriver::begin() {
    pinMode(pins::motor::IN1, OUTPUT);
    pinMode(pins::motor::IN2, OUTPUT);
    pinMode(pins::motor::IN3, OUTPUT);
    pinMode(pins::motor::IN4, OUTPUT);
    pinMode(pins::motor::ENA, OUTPUT);
    pinMode(pins::motor::ENB, OUTPUT);

    ledcSetup(kLeftChannel, 1000, kPwmResolution);
    ledcAttachPin(pins::motor::ENA, kLeftChannel);

    ledcSetup(kRightChannel, 1000, kPwmResolution);
    ledcAttachPin(pins::motor::ENB, kRightChannel);

    stop();
    applyState();
}

void MotorDriver::update(unsigned long currentMillis) {
    if (_dirty) {
        applyState();
    }
}

void MotorDriver::forward(uint8_t speed) {
    _direction = Direction::Forward;
    _speed = speed;
    _dirty = true;
}

void MotorDriver::backward(uint8_t speed) {
    _direction = Direction::Backward;
    _speed = speed;
    _dirty = true;
}

void MotorDriver::turnLeft(uint8_t speed) {
    
    _direction = Direction::TurnLeft;
    _speed = speed;
    _dirty = true;
}

void MotorDriver::turnRight(uint8_t speed) {
    _direction = Direction::TurnRight;
    _speed = speed;
    _dirty = true;
}

void MotorDriver::stop() {
    _direction = Direction::Stopped;
    _speed = 0;
    _dirty = true;
}

uint8_t MotorDriver::speed() const {
    return _speed;
}

MotorDriver::Direction MotorDriver::direction() const {
    return _direction;
}

void MotorDriver::driveWithCorrection(uint8_t speed, int8_t correction) {
    int16_t left  = static_cast<int16_t>(speed) - correction;
    int16_t right = static_cast<int16_t>(speed) + correction;
    left  = left  < 0 ? 0 : (left  > 255 ? 255 : left);
    right = right < 0 ? 0 : (right > 255 ? 255 : right);
    setMotorPower(left, right);
}

void MotorDriver::driveBackwardWithCorrection(uint8_t speed, int8_t correction) {
    // Ao recuar com motor esquerdo mais forte, o robô deriva para CCW (+yaw).
    // Correction > 0 → reduz motor esquerdo, aumenta direito → corrige CCW.
    int16_t left  = -(static_cast<int16_t>(speed) - correction);
    int16_t right = -(static_cast<int16_t>(speed) + correction);
    left  = left  < -255 ? -255 : (left  > 0 ? 0 : left);
    right = right < -255 ? -255 : (right > 0 ? 0 : right);
    setMotorPower(left, right);
}

void MotorDriver::applyState() {
    int16_t left = 0;
    int16_t right = 0;

    switch (_direction) {
        case Direction::Stopped:
            left = 0;
            right = 0;
            break;
        case Direction::Forward:
            left = _speed;
            right = _speed;
            break;
        case Direction::Backward:
            left = -static_cast<int16_t>(_speed);
            right = -static_cast<int16_t>(_speed);
            break;
        case Direction::TurnLeft:
            left = -static_cast<int16_t>(_speed);
            right = _speed;
            break;
        case Direction::TurnRight:
            left = _speed;
            right = -static_cast<int16_t>(_speed);
            break;
    }

    setMotorPower(left, right);
    _dirty = false;
}

void MotorDriver::setMotorPower(int16_t leftPower, int16_t rightPower) {
    auto writeMotor = [](int pinA, int pinB, int channel, int16_t power) {
        if (power == 0) {
            digitalWrite(pinA, LOW);
            digitalWrite(pinB, LOW);
            ledcWrite(channel, 0);
            return;
        }

        if (power > 0) {
            digitalWrite(pinA, HIGH);
            digitalWrite(pinB, LOW);
        } else {
            digitalWrite(pinA, LOW);
            digitalWrite(pinB, HIGH);
            power = -power;
        }

        uint8_t duty = static_cast<uint8_t>(power > kMaxDuty ? kMaxDuty : power);
        ledcWrite(channel, duty);
    };

    writeMotor(pins::motor::IN1, pins::motor::IN2, kLeftChannel, leftPower);
    writeMotor(pins::motor::IN3, pins::motor::IN4, kRightChannel, rightPower);
}

} // namespace drivers
