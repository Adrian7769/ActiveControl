#include "ServoDriver.h"

static constexpr uint8_t FAULT_NONE_LOCAL      = 0x00;
static constexpr uint8_t FAULT_SERVO_INIT_LOCAL = 0x30;

constexpr uint8_t ServoDriver::SERVO_PINS[SERVO_COUNT];

ServoDriver::ServoDriver()
    : _state{{90, 90, 90, 90}}
    , _trim{0, 0, 0, 0}
    , _healthy(false)
    , _last_fault(FAULT_NONE_LOCAL)
{}

uint8_t ServoDriver::begin() {
    for (int i = 0; i < SERVO_COUNT; i++) {
        _servo[i].setPeriodHertz(PWM_HZ);
        _servo[i].attach(SERVO_PINS[i], PULSE_MIN_US, PULSE_MAX_US);
    }

    _healthy = true;
    _last_fault = FAULT_NONE_LOCAL;
    center();
    return FAULT_NONE_LOCAL;
}

void ServoDriver::tick() {
    // ESP32 LEDC hardware holds the PWM signal indefinitely.
    // No periodic work needed.
    //
    // Future: health monitoring, slew rate limiting,
    // or servo current sensing could go here.
}

void ServoDriver::setAngle(uint8_t index, float degrees) {
    if (!_healthy || index >= SERVO_COUNT) return;

    if (degrees < 0.0f)   degrees = 0.0f;
    if (degrees > 180.0f) degrees = 180.0f;

    _state.angle[index] = degrees;

    // Apply trim offset to the pulse, not the stored angle.
    // _state.angle stays as the logical command (what guidance asked for).
    // The trim shifts the physical output to compensate for servo variation.
    float adjusted = degrees + _trim[index];
    if (adjusted < 0.0f)   adjusted = 0.0f;
    if (adjusted > 180.0f) adjusted = 180.0f;

    int us = (int)(PULSE_MIN_US +
             (adjusted / 180.0f) * (PULSE_MAX_US - PULSE_MIN_US));
    _servo[index].writeMicroseconds(us);
}

void ServoDriver::setAll(float d0, float d1, float d2, float d3) {
    setAngle(0, d0);
    setAngle(1, d1);
    setAngle(2, d2);
    setAngle(3, d3);
}

void ServoDriver::center() {
    setAll(90.0f, 90.0f, 90.0f, 90.0f);
}

void ServoDriver::setTrim(uint8_t index, float degrees) {
    if (index >= SERVO_COUNT) return;
    // Clamp to a sane range — if you need more than 15 degrees
    // of trim, the servo horn is mounted wrong
    if (degrees < -15.0f) degrees = -15.0f;
    if (degrees > 15.0f)  degrees = 15.0f;
    _trim[index] = degrees;
    // Re-apply current angle so trim takes effect immediately
    setAngle(index, _state.angle[index]);
}

float ServoDriver::getTrim(uint8_t index) const {
    if (index >= SERVO_COUNT) return 0;
    return _trim[index];
}

void ServoDriver::clearAllTrim() {
    for (int i = 0; i < SERVO_COUNT; i++) {
        _trim[i] = 0;
    }
    // Re-apply current angles without trim
    setAll(_state.angle[0], _state.angle[1],
           _state.angle[2], _state.angle[3]);
}
