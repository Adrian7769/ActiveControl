#include "IMU.h"
#include <Arduino.h>
#include <Wire.h>

static constexpr uint8_t FAULT_NONE_LOCAL          = 0x00;
static constexpr uint8_t FAULT_BNO_NOT_FOUND_LOCAL = 0x10;
static constexpr uint8_t FAULT_BNO_TIMEOUT_LOCAL   = 0x11;

IMU::IMU()
    : _sensor(&Wire, 0x28)   // BNO055 default I2C address
    , _orientation{0, 0, 0}
    , _linear_accel{0, 0, 0}
    , _gyro{0, 0, 0}
    , _raw_accel{0, 0, 0}
    , _calibration{0, 0, 0, 0}
    , _healthy(false)
    , _last_fault(FAULT_NONE_LOCAL)
    , _last_update_ms(0)
{
}

uint8_t IMU::begin() {
    DFRobot_BNO055::eStatus_t status = _sensor.begin();

    if (status == DFRobot_BNO055::eStatusOK) {
        _healthy = true;
        _last_fault = FAULT_NONE_LOCAL;
        return FAULT_NONE_LOCAL;
    }

    _healthy = false;
    if (status == DFRobot_BNO055::eStatusErrDeviceNotDetect) {
        _last_fault = FAULT_BNO_NOT_FOUND_LOCAL;
    } else {
        _last_fault = FAULT_BNO_TIMEOUT_LOCAL;
    }
    return _last_fault;
}

void IMU::tick() {
    if (!_healthy) return;
    if ((millis() - _last_update_ms) < UPDATE_INTERVAL_MS) return;
    _last_update_ms = millis();
    
    DFRobot_BNO055::sEulAnalog_t eul = _sensor.getEul();
    if (_sensor.lastOperateStatus != DFRobot_BNO055::eStatusOK) {
        _healthy = false; _last_fault = FAULT_BNO_TIMEOUT_LOCAL;
        return;
    }
    
    DFRobot_BNO055::sQuaAnalog_t qua = _sensor.getQua();
    DFRobot_BNO055::sAxisAnalog_t grv = _sensor.getAxis(DFRobot_BNO055::eAxisGrv);
    
    // ---- DEBUG: print raw library returns ----
/*
 *
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 1000) {
        lastDebug = millis();
        Serial.print(F("DBG qua raw: w="));
        Serial.print(qua.w, 4); Serial.print(F(" x="));
        Serial.print(qua.x, 4); Serial.print(F(" y="));
        Serial.print(qua.y, 4); Serial.print(F(" z="));
        Serial.println(qua.z, 4);
        
        Serial.print(F("DBG grv raw: x="));
        Serial.print(grv.x, 4); Serial.print(F(" y="));
        Serial.print(grv.y, 4); Serial.print(F(" z="));
        Serial.println(grv.z, 4);
        
        Serial.print(F("DBG qua status: "));
        Serial.println((int)_sensor.lastOperateStatus);
    }
 *
 * */

    // ---- END DEBUG ----
    
    DFRobot_BNO055::sAxisAnalog_t lia = _sensor.getAxis(DFRobot_BNO055::eAxisLia);
    DFRobot_BNO055::sAxisAnalog_t gyr = _sensor.getAxis(DFRobot_BNO055::eAxisGyr);
    DFRobot_BNO055::sAxisAnalog_t acc = _sensor.getAxis(DFRobot_BNO055::eAxisAcc);
    DFRobot_BNO055::sRegCalibState_t cal = _sensor.getCalStatus();
    
    _orientation = { eul.head, eul.roll, eul.pitch };
    _quaternion = { qua.w, qua.x, qua.y, qua.z };
    _linear_accel = { lia.x, lia.y, lia.z };
    _gravity = { grv.x, grv.y, grv.z };
    _gyro = { gyr.x, gyr.y, gyr.z };
    _raw_accel = { acc.x, acc.y, acc.z };
    _calibration = { cal.SYS, cal.GYR, cal.ACC, cal.MAG };
}
