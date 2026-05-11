#pragma once

#include <stdint.h>

/// GPIO mapping for the ESP32-based autonomous robot.
///
/// This file centralizes all pin definitions used by the robot hardware,
/// keeping the mapping easy to maintain and extend.
namespace pins {

/// Motor driver pin assignments for H-bridge control.
namespace motor {
    /// Left motor direction pin A.
    constexpr int IN1 = 13;
    /// Left motor direction pin B.
    constexpr int IN2 = 12;
    /// Right motor direction pin A.
    constexpr int IN3 = 14;
    /// Right motor direction pin B.
    constexpr int IN4 = 27;

    /// Left motor PWM enable.
    constexpr int ENA = 25;
    /// Right motor PWM enable.
    constexpr int ENB = 26;
}

/// Flame sensor pin.
namespace sensors {
    /// Fire sensor analog/digital input.
    constexpr int FLAME = 34;

    namespace ultrasonic {
        /// Ultrasonic sensor trigger output.
        constexpr int TRIG = 22;
        /// Ultrasonic sensor echo input.
        constexpr int ECHO = 23;
    }
}

/// Servo motor pin assignments.
namespace servo {
    /// Horizontal servo control.
    constexpr int HORIZONTAL = 18;
    /// Vertical servo control.
    constexpr int VERTICAL = 19;
}

/// Pump relay control pin.
namespace pump {
    /// Relay output for the water pump.
    constexpr int RELAY = 33;
}

} // namespace pins
