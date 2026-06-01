#include "ServoDriver.h"

static constexpr uint8_t FAULT_NONE_LOCAL      = 0x00;
static constexpr uint8_t FAULT_SERVO_INIT_LOCAL = 0x30;

constexpr uint8_t ServoDriver::SERVO_PINS[SERVO_COUNT];

ServoDriver::ServoDriver()
    : _state{{90, 90, 90, 90}}
    , _healthy(false)
    , _last_fault(FAULT_NONE_LOCAL)
{}

uint8_t ServoDriver::begin() {
    for (int i = 0; i < SERVO_COUNT; i++) {
        _servo[i].setPeriodHertz(PWM_HZ);
        if (!_servo[i].attach(SERVO_PINS[i], PULSE_MIN_US, PULSE_MAX_US)) {
            _healthy = false;
            _last_fault = FAULT_SERVO_INIT_LOCAL;
            return _last_fault;
        }
    }

    _healthy = true;
    _last_fault = FAULT_NONE_LOCAL;
    center();
    return FAULT_NONE_LOCAL;
}

void ServoDriver::tick() {
    // Future: health monitoring, slew rate limiting,
    // or servo current sensing could go here.
}

void ServoDriver::setAngle(uint8_t index, float degrees) {
    if (!_healthy || index >= SERVO_COUNT) return;

    if (degrees < 0.0f)   degrees = 0.0f;
    if (degrees > 180.0f) degrees = 180.0f;

    _state.angle[index] = degrees;

    int us = (int)(PULSE_MIN_US +
             (degrees / 180.0f) * (PULSE_MAX_US - PULSE_MIN_US));
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
