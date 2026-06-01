#include <Arduino.h>
#include <Wire.h>
#include <Barometer.h>
#include <IMU.h>
#include "config.h"
#include "fram.h"
#include "commands.h"
#include "faults.h"
#include <ServoDriver.h>
#include "Guidance.h"
FRAM        Fram;
IMU         Imu;
Barometer   Baro;
ServoDriver Servos;
Guidance    Guide(&Imu, &Servos);
Commands    Cmd(&Fram, &Imu, &Baro, &Servos, &Guide);
DataBlock   Block;

// general flags
static constexpr float  LAUNCH_ACCEL_MG      = 3000.0f;  // 3G in milli-g
static constexpr unsigned long LAUNCH_HOLD_MS = 100;      // sustained for 100ms
static constexpr unsigned long APOGEE_HOLD_MS = 500;      // altitude falling for 500ms

// detect launch flags
static unsigned long launchAccelStart = 0;
static bool          launchDetected   = false;
// detect appogee flags1
static float         peakAltitude     = -9999.0f;
static unsigned long altFallingStart  = 0;
// logging rate to FRAM
static constexpr unsigned long LOG_INTERVAL_MS = 100;  // 10 Hz
// Pack the data into 32 byte block
void packDataBlock() {
    Block.timestamp_ms = millis();
    Block.pressure_pa = Baro.isHealthy() ? (uint32_t)Baro.getPressurePa() : 0;
    Quaternion q = Imu.getQuaternion();
    Block.qw = (int16_t)(q.w * 16384.0f);
    Block.qx = (int16_t)(q.x * 16384.0f);
    Block.qy = (int16_t)(q.y * 16384.0f);

    Block.qz = (int16_t)(q.z * 16384.0f);
    Vec3 a = Imu.getLinearAccel();
    Block.accel_x = (int16_t)a.x;
    Block.accel_y = (int16_t)a.y;
    Block.accel_z = (int16_t)a.z;
    Block.pid_pitch = (int8_t)(Guide.getPitchCmd() * 100.0f);
    Block.pid_yaw   = (int8_t)(Guide.getYawCmd()   * 100.0f);
    Block.pid_roll  = (int8_t)(Guide.getRollCmd()   * 100.0f);
    ServoState s = Servos.getState();
    Block.fin_0 = (uint8_t)s.angle[0];
    Block.fin_1 = (uint8_t)s.angle[1];
    Block.fin_2 = (uint8_t)s.angle[2];
    Block.fin_3 = (uint8_t)s.angle[3];
    Block.flight_state = Fram.GetProgramState();
    Block.flags = 0;
    if (Guide.isEnabled())      Block.flags |= (1 << 0);
    if (Fram.IsFull())          Block.flags |= (1 << 1);
    if (!Imu.isHealthy())       Block.flags |= (1 << 2);
    if (!Baro.isHealthy())      Block.flags |= (1 << 3);
    if (!Servos.isHealthy())    Block.flags |= (1 << 4);
    Block._reserved = 0;
}
bool checkLaunchDetected() {
    Vec3 a = Imu.getLinearAccel();
    float accel_mag = sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
    if (accel_mag >= LAUNCH_ACCEL_MG) {
        if (launchAccelStart == 0) {
            launchAccelStart = millis();
        } else if (millis() - launchAccelStart >= LAUNCH_HOLD_MS) {
            return true;
        }
    } else {
        launchAccelStart = 0;
    }
    return false;
}
bool checkApogeeDetected() {
    if (!Baro.isHealthy()) return false;

    float alt = Baro.getAltitudeM();
    if (alt > peakAltitude) {
        peakAltitude = alt;
        altFallingStart = 0;
    } else {
        if (altFallingStart == 0) {
            altFallingStart = millis();
        } else if (millis() - altFallingStart >= APOGEE_HOLD_MS) {
            return true;
        }
    }
    return false;
}
void updateStateMachine() {
    uint8_t state = Fram.GetProgramState();
    switch (static_cast<ProgramState>(state)) {
        case ProgramState::ARMED:
            if (checkLaunchDetected()) {
                Fram.SetProgramState(static_cast<uint8_t>(ProgramState::ASCENT));
                Guide.enable();
                peakAltitude = Baro.getAltitudeM();
                launchDetected = true;
#ifdef DIAG_COM
                Serial.println(F("LAUNCH DETECTED — ASCENT"));
#endif
            }
            break;

        case ProgramState::ASCENT:
            if (checkApogeeDetected()) {
                Fram.SetProgramState(static_cast<uint8_t>(ProgramState::APOGEE));
                Guide.disable();
#ifdef DIAG_COM
                Serial.println(F("APOGEE DETECTED"));
#endif
            }
            break;

        case ProgramState::APOGEE:
            Fram.SetProgramState(static_cast<uint8_t>(ProgramState::DESCENT));
            break;

        case ProgramState::DESCENT:
            break;

        default:
            break;
    }
}
void setup() {
    Serial.begin(115200);
    delay(100);  
    Wire.begin(ESP_SDA_PIN, ESP_SCL_PIN);
    Wire.setClock(400000);  // 400 kHz 
    Fram.begin();
    uint8_t fault;
    fault = Baro.begin();
    if (fault != FAULT_NONE) {
        Fram.SetErrorCodeByte(fault);
        Fram.SetProgramState(static_cast<uint8_t>(ProgramState::FAULT));
        Serial.print(F("Barometer fault: 0x")); Serial.println(fault, HEX);
    }
    fault = Imu.begin();
    if (fault != FAULT_NONE) {
        Fram.SetErrorCodeByte(fault);
        Fram.SetProgramState(static_cast<uint8_t>(ProgramState::FAULT));
        Serial.print(F("IMU fault: 0x")); Serial.println(fault, HEX);
    }
    fault = Servos.begin();
    if (fault != FAULT_NONE) {
        Fram.SetErrorCodeByte(fault);
        Fram.SetProgramState(static_cast<uint8_t>(ProgramState::FAULT));
        Serial.print(F("Servo fault: 0x")); Serial.println(fault, HEX);
    }
    Guide.begin();
    if (Fram.GetErrorCodeByte() != FAULT_NONE) {
        bool all_ok = Baro.isHealthy() && Imu.isHealthy() && Servos.isHealthy();
        if (all_ok) {
            Fram.SetErrorCodeByte(FAULT_NONE);
            if (Fram.GetProgramState() == static_cast<uint8_t>(ProgramState::FAULT)) {
                Fram.SetProgramState(static_cast<uint8_t>(ProgramState::IDLE));
                Serial.println(F("Previous fault cleared, all systems healthy."));
            }
        }
    }
    uint8_t prev_state = Fram.GetProgramState();
    if (prev_state == static_cast<uint8_t>(ProgramState::UNINITIALIZED)) {
        Fram.SetProgramState(static_cast<uint8_t>(ProgramState::IDLE));
    } else if (prev_state >= static_cast<uint8_t>(ProgramState::ARMED) &&
               prev_state <= static_cast<uint8_t>(ProgramState::LANDED)) {
        Fram.SetProgramState(static_cast<uint8_t>(ProgramState::POST_FLIGHT));
        Serial.println(F("WARNING: interrupted flight detected"));
    }

    Cmd.begin();
}
void loop() {
    Cmd.tick();
    Baro.tick();
    Imu.tick();
    Servos.tick();
    Guide.tick();
    updateStateMachine();
    static unsigned long lastLog = 0;
    unsigned long now = millis();
    if (now - lastLog >= LOG_INTERVAL_MS) {
        lastLog = now;
        uint8_t state = Fram.GetProgramState();
        // Only log during active flight states
        if (state >= static_cast<uint8_t>(ProgramState::ARMED) &&
            state <= static_cast<uint8_t>(ProgramState::LANDED)) {
            packDataBlock();
            Fram.WriteDataBlock(&Block);
        }
    }
}
