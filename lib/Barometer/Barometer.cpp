#include "Barometer.h"
#include <Arduino.h>
#include <Wire.h>

static constexpr uint8_t FAULT_NONE_LOCAL          = 0x00;
static constexpr uint8_t FAULT_BMP_NOT_FOUND_LOCAL = 0x20;
static constexpr uint8_t FAULT_BMP_TIMEOUT_LOCAL   = 0x21;

Barometer::Barometer()
    : _sensor(&Wire, DFRobot_BMP280_IIC::eSdoLow)   // SDO low = address 0x76
    , _pressure_pa(0)
    , _temperature_c(0)
    , _altitude_m(0)
    , _sea_level_pa(101325.0f)
    , _healthy(false)
    , _last_fault(FAULT_NONE_LOCAL)
    , _last_update_ms(0)
{
}

uint8_t Barometer::begin() {
    DFRobot_BMP280::eStatus_t status = _sensor.begin();

    if (status == DFRobot_BMP280::eStatusOK) {
        _healthy = true;
        _last_fault = FAULT_NONE_LOCAL;
        return FAULT_NONE_LOCAL;
    }

    _healthy = false;
    if (status == DFRobot_BMP280::eStatusErrDeviceNotDetected) {
        _last_fault = FAULT_BMP_NOT_FOUND_LOCAL;
    } else {
        _last_fault = FAULT_BMP_TIMEOUT_LOCAL;
    }
    return _last_fault;
}

void Barometer::tick() {
    if (!_healthy) return;
    if ((millis() - _last_update_ms) < UPDATE_INTERVAL_MS) return;
    _last_update_ms = millis();

    uint32_t pressure_raw = _sensor.getPressure();
    
    // Check if the read succeeded
    if (_sensor.lastOperateStatus != DFRobot_BMP280::eStatusOK) {
        _healthy = false;
        _last_fault = FAULT_BMP_TIMEOUT_LOCAL;
        return;
    }

    float temp = _sensor.getTemperature();
    if (_sensor.lastOperateStatus != DFRobot_BMP280::eStatusOK) {
        _healthy = false;
        _last_fault = FAULT_BMP_TIMEOUT_LOCAL;
        return;
    }

    // Cache the readings
    _pressure_pa = static_cast<float>(pressure_raw);
    _temperature_c = temp;
    _altitude_m = _sensor.calAltitude(_sea_level_pa, pressure_raw);
}
