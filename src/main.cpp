#include <Arduino.h>
#include <Wire.h>
#include <Barometer.h>
#include <IMU.h>
#include "config.h"
#include "fram.h"
#include "commands.h"
#include "faults.h"

// ---- Globals ----
FRAM      Fram;
IMU       Imu;
Barometer Baro;
Commands  Cmd(&Fram, &Imu, &Baro);

void setup() {
    Serial.begin(115200);
    delay(2000);
    Wire.begin(ESP_SDA_PIN, ESP_SCL_PIN);

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

    // Initial state logic
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

void printSensors() {
    Serial.println(F("=========================================="));

    // Barometer
    if (Baro.isHealthy()) {
        Serial.print(F("Press: ")); Serial.print(Baro.getPressurePa(), 1); Serial.print(F(" Pa  "));
        Serial.print(F("Temp: "));  Serial.print(Baro.getTemperatureC(), 2); Serial.print(F(" C  "));
        Serial.print(F("Alt: "));   Serial.print(Baro.getAltitudeM(), 2);    Serial.println(F(" m"));
    } else {
        Serial.println(F("Baro: FAULT"));
    }

    if (!Imu.isHealthy()) {
        Serial.println(F("IMU: FAULT"));
        return;
    }

    Orientation       o = Imu.getOrientation();
    Quaternion        q = Imu.getQuaternion();
    Vec3              a = Imu.getLinearAccel();
    Vec3              g = Imu.getGravityVector();
    Vec3              gyr = Imu.getGyro();
    Vec3              raw = Imu.getRawAccel();
    CalibrationStatus c = Imu.getCalibration();

    Serial.print(F("Euler  (deg)    : head=")); Serial.print(o.heading, 1);
    Serial.print(F("  roll="));                 Serial.print(o.roll,    1);
    Serial.print(F("  pitch="));                Serial.println(o.pitch, 1);

    Serial.print(F("Quat            : w=")); Serial.print(q.w, 3);
    Serial.print(F(" x="));                  Serial.print(q.x, 3);
    Serial.print(F(" y="));                  Serial.print(q.y, 3);
    Serial.print(F(" z="));                  Serial.println(q.z, 3);

    Serial.print(F("LinAcc (mg)     : x=")); Serial.print(a.x, 1);
    Serial.print(F("  y="));                 Serial.print(a.y, 1);
    Serial.print(F("  z="));                 Serial.println(a.z, 1);

    Serial.print(F("Gravity (mg)    : x=")); Serial.print(g.x, 1);
    Serial.print(F("  y="));                 Serial.print(g.y, 1);
    Serial.print(F("  z="));                 Serial.println(g.z, 1);

    Serial.print(F("Gyro (dps)      : x=")); Serial.print(gyr.x, 1);
    Serial.print(F("  y="));                 Serial.print(gyr.y, 1);
    Serial.print(F("  z="));                 Serial.println(gyr.z, 1);

    Serial.print(F("RawAcc (mg)     : x=")); Serial.print(raw.x, 1);
    Serial.print(F("  y="));                 Serial.print(raw.y, 1);
    Serial.print(F("  z="));                 Serial.println(raw.z, 1);

    Serial.print(F("Calib SYS/G/A/M : "));
    Serial.print(c.system); Serial.print('/');
    Serial.print(c.gyro);   Serial.print('/');
    Serial.print(c.accel);  Serial.print('/');
    Serial.println(c.mag);

    // Derived useful quantities
    float tilt_from_vertical = acos(g.z / 1000.0f) * 180.0f / PI;
    Serial.print(F("Tilt from vert  : ")); Serial.print(tilt_from_vertical, 1); Serial.println(F(" deg"));

    float total_g = sqrt(raw.x*raw.x + raw.y*raw.y + raw.z*raw.z) / 1000.0f;
    Serial.print(F("Total accel mag : ")); Serial.print(total_g, 2); Serial.println(F(" G"));
}

void loop() {
    Cmd.tick();
    Baro.tick();
    Imu.tick();

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 1000) {
        lastPrint = millis();
        printSensors();
    }
}
