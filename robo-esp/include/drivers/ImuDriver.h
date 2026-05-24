#pragma once

#include <stdint.h>

namespace drivers {

class ImuDriver {
public:
    bool begin(int sdaPin, int sclPin);
    void update();

    float yaw()     const;
    float yawRate() const;
    void  resetYaw();
    bool  isReady() const;

    /// Inverte sinal do eixo Z (caso o chip esteja montado de cabeça para baixo).
    void setYawInverted(bool inverted);

    /// Coeficiente do filtro EMA aplicado à taxa de rotação (yaw rate).
    /// Range [0.0, 1.0]:
    ///   1.0 = sem filtro (máxima responsividade, mais ruído)
    ///   0.7 = equilíbrio recomendado para robôs lentos/médios  ← padrão
    ///   0.4 = muita suavização (lag maior; pode subestimar pico de rotação)
    void setFilterAlpha(float alpha);

private:
    static const uint8_t kAddress    = 0x68;
    // Sensibilidade no modo ±500°/s: 65,5 LSB por (°/s)
    static constexpr float kSensitivity = 1.0f / 65.5f;

    bool readRawGyroZ(int16_t& rawZ);
    void calibrate();
    void writeReg(uint8_t reg, uint8_t value);

    float         _yaw              = 0.0f;
    float         _yawRate          = 0.0f;  // taxa filtrada (EMA)
    float         _biasDegsPerSec   = 0.0f;
    float         _filterAlpha      = 0.7f;  // coeficiente EMA
    unsigned long _lastMicros       = 0;
    bool          _ready            = false;
    bool          _inverted         = false;
};

} // namespace drivers
