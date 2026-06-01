#include "ServoDriver.h"
#include <Arduino.h>
#include <Wire.h>

// Keep local copies so this compiles standalone like your other drivers
static constexpr uint8_t  FAULT_NONE_LOCAL     = 0x00;
static constexpr uint8_t  FAULT_PCA_NF_LOCAL   = 0x30;
static constexpr uint8_t  FAULT_PCA_T_LOCAL    = 0x31;

static constexpr uint8_t  PCA_ADDR       = 0x40;
static constexpr float    PWM_FREQ       = 50.0f;
static constexpr uint16_t PULSE_MIN      = 150;
static constexpr uint16_t PULSE_MAX      = 600;

// Map servo index 0-3 to PCA9685 channel pins
static constexpr uint8_t DEFAULT_MAP[4] = { 0, 1, 4, 5 };

ServoDriver::ServoDriver()
    : _pca(PCA_ADDR)
    , _state{{90, 90, 90, 90}}
    , _healthy(false)
    , _last_fault(FAULT_NONE_LOCAL)
{
    memcpy(_channel_map, DEFAULT_MAP, sizeof(_channel_map));
}

uint8_t ServoDriver::begin() {
    // Probe the bus first — PCA9685 lives at PCA_ADDR
    Wire.beginTransmission(PCA_ADDR);
    uint8_t err = Wire.endTransmission();
    if (err != 0) {
        _healthy = false;
        _last_fault = FAULT_PCA_NF_LOCAL;
        return _last_fault;
    }

    _pca.begin();
    _pca.setPWMFreq(PWM_FREQ);
    delay(10);

    _healthy = true;
    _last_fault = FAULT_NONE_LOCAL;

    center();  // safe starting position
    return FAULT_NONE_LOCAL;
}

void ServoDriver::tick() {
    // Nothing periodic needed for the PCA9685.
    // Servos hold position until you send a new command.
    // You could add watchdog / heartbeat probing here later.
}

void ServoDriver::setAngle(uint8_t index, float degrees) {
    if (!_healthy || index >= 4) return;

    degrees = constrain(degrees, 0.0f, 180.0f);
    _state.angle[index] = degrees;
    _pca.setPWM(_channel_map[index], 0, angleToPulse(degrees));
}

void ServoDriver::setAll(float d0, float d1, float d2, float d3) {
    setAngle(0, d0);
    setAngle(1, d1);
    setAngle(2, d2);
    setAngle(3, d3);
}

void ServoDriver::center() {
    setAll(90, 90, 90, 90);
}

uint16_t ServoDriver::angleToPulse(float degrees) {
    return (uint16_t)map((long)(degrees * 10), 0, 1800,
                         PULSE_MIN, PULSE_MAX);
}
