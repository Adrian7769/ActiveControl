#include "commands.h"
#include <string.h>
#include <ctype.h>
#include "config.h"
#include "faults.h"

Commands::Commands(FRAM* fram, IMU* Imu, Barometer* Baro, ServoDriver* servo, Guidance* guide) {
    _fram = fram;
    _baro = Baro;
    _imu = Imu;
    _servo = servo;
    _guide = guide;
    _pos = 0;
    _echo = true;
    _buffer[0] = '\0';
    _monMode = MonitorMode::NONE;
    _monLastMs = 0;
    _monIntervalMs = 200; // 5 Hz default
}

void Commands::begin() {
    printWelcome();
    printPrompt();
}

void Commands::tick() {
    // If monitoring handle that instead of normal command processing
    if (_monMode != MonitorMode::NONE) {
        monTick();
        return;
    }

    if (readLine()) {
        dispatch(&_buffer[0]);
        resetBuffer();
        printPrompt();
    }
}

bool Commands::readLine() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            if (_echo) Serial.println();
            _buffer[_pos] = '\0';
            return true; 
        }
        if (c == '\b') {
            if (_pos > 0) {
                _pos--;
                if (_echo) Serial.print("\b \b");
            }
            continue;
        }
        if (_pos < CMD_BUFFER_SIZE - 1) {
            _buffer[_pos++] = c;
            if (_echo) Serial.write(c);
        }
    }
    return false;
}

void Commands::resetBuffer() {
    _pos = 0;
    _buffer[0] = '\0';
}

void Commands::dispatch(char* line) {
    char* verb = strtok(line, " \t");
    char* arg  = strtok(nullptr, "");

    if (verb == nullptr) return;
    while (arg && *arg == ' ') arg++;

    // System
    if      (strcasecmp(verb, "STATUS")    == 0) cmdStatus();
    else if (strcasecmp(verb, "HELP")      == 0) {
        if (arg != nullptr && strcasecmp(arg, "FAULT") == 0) cmdHelpFaults();
        else cmdHelp();
    }
    else if (strcasecmp(verb, "?")         == 0) cmdHelp();
    else if (strcasecmp(verb, "DUMP")      == 0) cmdDump(arg);
    else if (strcasecmp(verb, "CLEAR")     == 0) cmdClear(arg);
    else if (strcasecmp(verb, "ECHO")      == 0) cmdEcho(arg);
    // Sensors
    else if (strcasecmp(verb, "SENSORS")   == 0) cmdSensors();
    else if (strcasecmp(verb, "CALIBRATE") == 0) cmdCal();
    // Actuators
    else if (strcasecmp(verb, "SERVO")     == 0) cmdServo(arg);
    else if (strcasecmp(verb, "CENTER")    == 0) cmdCenter();
    else if (strcasecmp(verb, "TRIM")      == 0) cmdTrim(arg);
    // Guidance
    else if (strcasecmp(verb, "GUIDE")     == 0) cmdGuide(arg);
    else if (strcasecmp(verb, "GAINS")     == 0) cmdGains(arg);
    // Flight
    else if (strcasecmp(verb, "ARM")       == 0) cmdArm();
    else if (strcasecmp(verb, "DISARM")    == 0) cmdDisarm();
    // Monitoring
    else if (strcasecmp(verb, "MONITOR")   == 0) cmdMonitor(arg);
    else if (strcasecmp(verb, "MON")       == 0) cmdMonitor(arg);
    else cmdUnknown(verb);
}

void Commands::printWelcome() {
    Serial.println();
    Serial.println(F("================================"));
    Serial.print(F("  Flight Computer  v")); Serial.println(VERSION);
    Serial.println(F("================================"));
    Serial.print(F("State:   "));
    Serial.println(stateName(static_cast<ProgramState>(_fram->GetProgramState())));
    Serial.print(F("Records: ")); Serial.println(_fram->GetRecordCount());
    Serial.println();
    Serial.println(F("Type HELP or ? for commands."));
    Serial.println();
}

void Commands::printPrompt() {
    Serial.print('[');
    Serial.print(stateName(static_cast<ProgramState>(_fram->GetProgramState())));
    Serial.print(F("]> "));
}

const char* Commands::stateName(ProgramState s) {
    switch (s) {
        case ProgramState::UNINITIALIZED: return "UNINIT";
        case ProgramState::IDLE:          return "IDLE";
        case ProgramState::FAULT:         return "FAULT";
        case ProgramState::ARMED:         return "ARMED";
        case ProgramState::ASCENT:        return "ASCENT";
        case ProgramState::APOGEE:        return "APOGEE";
        case ProgramState::DESCENT:       return "DESCENT";
        case ProgramState::LANDED:        return "LANDED";
        case ProgramState::POST_FLIGHT:   return "POST_FLIGHT";
        default:                          return "N/A";
    }
}

void Commands::monTick() {
    if (Serial.available()) {
        while (Serial.available()) Serial.read(); // flush
        monStop();
        return;
    }

    if (millis() - _monLastMs < _monIntervalMs) return;
    _monLastMs = millis();

    switch (_monMode) {
        case MonitorMode::SENSORS: monPrintSensors(); break;
        case MonitorMode::SERVOS:  monPrintServos();  break;
        case MonitorMode::PID:     monPrintPID();     break;
        case MonitorMode::GUIDE:   monPrintGuide();   break;
        case MonitorMode::ALL:
            monPrintSensors();
            monPrintServos();
            monPrintPID();
            Serial.println();
            break;
        default: break;
    }
}

void Commands::monPrintSensors() {
    if (_baro != nullptr && _baro->isHealthy()) {
        Serial.print(F("ALT:"));
        Serial.print(_baro->getAltitudeM(), 1);
        Serial.print(F("m "));
    }
    if (_imu != nullptr && _imu->isHealthy()) {
        Quaternion q = _imu->getQuaternion();
        Vec3 a = _imu->getLinearAccel();
        Serial.print(F("Q:")); 
        Serial.print(q.w, 3); Serial.print(F(","));
        Serial.print(q.x, 3); Serial.print(F(","));
        Serial.print(q.y, 3); Serial.print(F(","));
        Serial.print(q.z, 3);
        Serial.print(F(" A:"));
        Serial.print(a.x, 0); Serial.print(F(","));
        Serial.print(a.y, 0); Serial.print(F(","));
        Serial.print(a.z, 0);
    }
    Serial.println();
}

void Commands::monPrintServos() {
    if (_servo == nullptr || !_servo->isHealthy()) return;
    ServoState s = _servo->getState();
    Serial.print(F("FIN: "));
    for (int i = 0; i < 4; i++) {
        Serial.print(s.angle[i], 1);
        float t = _servo->getTrim(i);
        if (t != 0.0f) {
            Serial.print(F("["));
            if (t > 0) Serial.print('+');
            Serial.print(t, 0);
            Serial.print(F("]"));
        }
        if (i < 3) Serial.print(F("  "));
    }
    Serial.println();
}

void Commands::monPrintPID() {
    if (_guide == nullptr) return;
    Serial.print(F("PID: P="));
    Serial.print(_guide->getPitchCmd(), 3);
    Serial.print(F("  Y="));
    Serial.print(_guide->getYawCmd(), 3);
    Serial.print(F("  R="));
    Serial.print(_guide->getRollCmd(), 3);
    Serial.print(F("  ["));
    Serial.print(_guide->isEnabled() ? F("ON") : F("OFF"));
    Serial.println(F("]"));
}

void Commands::monPrintGuide() {
    // Full guidance view: quaternion + PID + fins
    if (_imu != nullptr && _imu->isHealthy()) {
        Quaternion q = _imu->getQuaternion();
        Serial.print(F("Q:"));
        Serial.print(q.w, 3); Serial.print(F(","));
        Serial.print(q.x, 3); Serial.print(F(","));
        Serial.print(q.y, 3); Serial.print(F(","));
        Serial.print(q.z, 3);
    }
    if (_guide != nullptr) {
        Serial.print(F(" -> P:"));
        Serial.print(_guide->getPitchCmd(), 2);
        Serial.print(F(" Y:"));
        Serial.print(_guide->getYawCmd(), 2);
        Serial.print(F(" R:"));
        Serial.print(_guide->getRollCmd(), 2);
    }
    if (_servo != nullptr && _servo->isHealthy()) {
        ServoState s = _servo->getState();
        Serial.print(F(" -> F:"));
        Serial.print(s.angle[0], 0); Serial.print(F(","));
        Serial.print(s.angle[1], 0); Serial.print(F(","));
        Serial.print(s.angle[2], 0); Serial.print(F(","));
        Serial.print(s.angle[3], 0);
    }
    Serial.println();
}

void Commands::monStop() {
    _monMode = MonitorMode::NONE;
    Serial.println();
    Serial.println(F("Monitor stopped."));
    printPrompt();
}

void Commands::cmdHelp() {
    Serial.println(F(""));
    Serial.println(F("--- SYSTEM ---"));
    Serial.println(F("  STATUS              state, uptime, records"));
    Serial.println(F("  HELP / ?            this list"));
    Serial.println(F("  HELP FAULT          fault code reference"));
    Serial.println(F("  DUMP [csv|raw]      dump FRAM records (default: csv)"));
    Serial.println(F("  CLEAR CONFIRM       erase all records"));
    Serial.println(F("  ECHO on|off         toggle character echo"));
    //Serial.println(F(""));
    Serial.println(F("--- SENSORS ---"));
    Serial.println(F("  SENSORS             snap-shot sensor readout"));
    Serial.println(F("  CALIBRATE           BNO055 calibration status"));
    //Serial.println(F(""));
    Serial.println(F("--- ACTUATORS ---"));
    Serial.println(F("  SERVO [n] [angle]   show/set fin angle"));
    Serial.println(F("  CENTER              center all fins (90 deg)"));
    Serial.println(F("  TRIM [n] [offset]   show/set per-servo trim"));
    Serial.println(F("  TRIM CLEAR          zero all trim values"));
    //Serial.println(F(""));
    Serial.println(F("--- GUIDANCE ---"));
    Serial.println(F("  GUIDE on|off        enable/disable guidance"));
    Serial.println(F("  GAINS p|y|r kp ki kd  set PID gains"));
    //Serial.println(F(""));
    Serial.println(F("--- FLIGHT ---"));
    Serial.println(F("  ARM                 IDLE -> ARMED"));
    Serial.println(F("  DISARM              ARMED -> IDLE"));
    //Serial.println(F(""));
    Serial.println(F("--- MONITORING (any key to stop) ---"));
    Serial.println(F("  MON sensors         quaternion, accel, altitude"));
    Serial.println(F("  MON servos          fin positions + trim"));
    Serial.println(F("  MON pid             PID outputs (P/Y/R)"));
    Serial.println(F("  MON guide           quat -> PID -> fins pipeline"));
    Serial.println(F("  MON all             sensors + servos + PID"));
    Serial.println(F("  MON <mode> <ms>     set refresh rate (default 200)"));
    Serial.println(F(""));
}

void Commands::cmdHelpFaults() {
    Serial.println(F("--- FAULT CODES ---"));
    Serial.println(F("Code  Name                  Description"));
    Serial.println(F("----  --------------------  --------------------------------"));

    for (size_t i = 0; i < FAULT_TABLE_SIZE; ++i) {
        const FaultEntry& f = FAULT_TABLE[i];

        Serial.print(F("0x"));
        if (f.code < 0x10) Serial.print('0');
        Serial.print(f.code, HEX);
        Serial.print(F("  "));

        Serial.print(f.name);
        for (size_t pad = strlen(f.name); pad < 20; ++pad) Serial.print(' ');

        Serial.print(F("  "));
        Serial.println(f.description);
    }

    Serial.println(F("-------------------"));

    uint8_t current = _fram->GetErrorCodeByte();
    Serial.print(F("Current error code: 0x"));
    if (current < 0x10) Serial.print('0');
    Serial.print(current, HEX);

    const FaultEntry* entry = lookupFault(current);
    if (entry) {
        Serial.print(F("  (")); Serial.print(entry->name); Serial.println(F(")"));
    } else {
        Serial.println(F("  (unknown)"));
    }
}

void Commands::cmdStatus() {
    Serial.println(F("--- STATUS ---"));
    Serial.print(F("State:        "));
    Serial.println(stateName(static_cast<ProgramState>(_fram->GetProgramState())));
    Serial.print(F("Uptime:       ")); Serial.print(millis() / 1000); Serial.println(F(" s"));
    Serial.print(F("Records:      ")); Serial.println(_fram->GetRecordCount());
    Serial.print(F("Memory:       "));
        Serial.print((_fram->GetRecordCount() * 100UL) / FRAM_MAX_RECORD);
        Serial.println(F("% full"));
    Serial.print(F("Last error:   0x"));
        uint8_t err = _fram->GetErrorCodeByte();
        if (err < 0x10) Serial.print('0');
        Serial.println(err, HEX);
    Serial.print(F("Echo:         ")); Serial.println(_echo ? "ON" : "OFF");
    Serial.print(F("Guidance:     "));
    if (_guide != nullptr) {
        Serial.println(_guide->isEnabled() ? "ENABLED" : "DISABLED");
    } else {
        Serial.println(F("N/A"));
    }
    Serial.print(F("Servos:       "));
    if (_servo != nullptr && _servo->isHealthy()) {
        ServoState s = _servo->getState();
        Serial.print(s.angle[0], 0); Serial.print(F(" "));
        Serial.print(s.angle[1], 0); Serial.print(F(" "));
        Serial.print(s.angle[2], 0); Serial.print(F(" "));
        Serial.println(s.angle[3], 0);
    } else {
        Serial.println(F("N/A"));
    }
    Serial.println(F("--------------"));
}

void Commands::cmdDump(const char* arg) {
    uint16_t count = _fram->GetRecordCount();
    if (count == 0) {
        Serial.println(F("No records to dump."));
        return;
    }

    if (arg != nullptr && strcasecmp(arg, "CSV") == 0) {
        Serial.print(F("# IRIS Flight Data - "));
        Serial.print(count); Serial.println(F(" records"));
        _fram->DumpCSV();
        Serial.print(F("# END - ")); Serial.print(count);
        Serial.println(F(" records"));
    } else if (arg != nullptr && strcasecmp(arg, "RAW") == 0) {
        // Raw hex bytes
        Serial.println(F("Control block:"));
        _fram->DumpControlBlock();
        Serial.println(F("Data blocks:"));
        _fram->DumpDataBytes();
        Serial.println(F("Dump complete."));
    } else {
	    // csv
        Serial.print(F("# IRIS Flight Data - "));
        Serial.print(count); Serial.println(F(" records"));
        _fram->DumpCSV();
        Serial.print(F("# END - ")); Serial.print(count);
        Serial.println(F(" records"));
    }
}

void Commands::cmdClear(const char* arg) {
    uint8_t s = _fram->GetProgramState();

    if (s == static_cast<uint8_t>(ProgramState::ARMED)   ||
        s == static_cast<uint8_t>(ProgramState::ASCENT)  ||
        s == static_cast<uint8_t>(ProgramState::APOGEE)  ||
        s == static_cast<uint8_t>(ProgramState::DESCENT) ||
        s == static_cast<uint8_t>(ProgramState::LANDED)) {
        Serial.println(F("ERR: cannot clear during flight"));
        return;
    }

    uint16_t count = _fram->GetRecordCount();

    if (arg == nullptr || strcasecmp(arg, "CONFIRM") != 0) {
        Serial.println(F("WARNING: this will erase all flight data."));
        Serial.print(F("  ")); Serial.print(count);
        Serial.println(F(" records will be lost."));
        Serial.println();
        Serial.println(F("  Type CLEAR CONFIRM to proceed."));
        Serial.println(F("  Type DUMP first if you need the data."));
        return;
    }

    if (_guide != nullptr && _guide->isEnabled()) {
        _guide->disable();
    }

    Serial.print(F("Clearing ")); Serial.print(count);
    Serial.println(F(" records..."));
    _fram->ResetFram();
    Serial.println(F("FRAM cleared. State: IDLE."));
}

void Commands::cmdEcho(const char* arg) {
    if (arg == nullptr || *arg == '\0') {
        Serial.print(F("echo is "));
        Serial.println(_echo ? "ON" : "OFF");
        return;
    }
    if (strcasecmp(arg, "on") == 0) {
        _echo = true;
        Serial.println(F("echo ON"));
    } else if (strcasecmp(arg, "off") == 0) {
        _echo = false;
        Serial.println(F("echo OFF"));
    } else {
        Serial.println(F("usage: ECHO on|off"));
    }
}

void Commands::cmdSensors() {
    Serial.println(F("--- SENSORS ---"));

    Serial.print(F("Baro:   "));
    if (_baro != nullptr && _baro->isHealthy()) {
        Serial.print(_baro->getPressurePa(), 1); Serial.print(F(" Pa  "));
        Serial.print(_baro->getTemperatureC(), 2); Serial.print(F(" C  "));
        Serial.print(_baro->getAltitudeM(), 2); Serial.println(F(" m"));
    } else {
        Serial.print(F("FAULT 0x"));
        Serial.println(_baro ? _baro->getLastFault() : 0xFF, HEX);
    }

    Serial.print(F("IMU:    "));
    if (_imu != nullptr && _imu->isHealthy()) {
        Serial.println(F("OK"));
        Quaternion q = _imu->getQuaternion();
        Serial.print(F("  Quat:   "));
        Serial.print(q.w, 3); Serial.print(F(" "));
        Serial.print(q.x, 3); Serial.print(F(" "));
        Serial.print(q.y, 3); Serial.print(F(" "));
        Serial.println(q.z, 3);

        Orientation o = _imu->getOrientation();
        Serial.print(F("  Euler:  "));
        Serial.print(F("H:")); Serial.print(o.heading, 1);
        Serial.print(F(" R:")); Serial.print(o.roll, 1);
        Serial.print(F(" P:")); Serial.println(o.pitch, 1);

        Vec3 a = _imu->getLinearAccel();
        Serial.print(F("  Accel:  "));
        Serial.print(a.x, 1); Serial.print(F(" "));
        Serial.print(a.y, 1); Serial.print(F(" "));
        Serial.print(a.z, 1); Serial.println(F(" mg"));

        Vec3 g = _imu->getGravityVector();
        Serial.print(F("  Grav:   "));
        Serial.print(g.x, 1); Serial.print(F(" "));
        Serial.print(g.y, 1); Serial.print(F(" "));
        Serial.print(g.z, 1); Serial.println(F(" mg"));

        CalibrationStatus c = _imu->getCalibration();
        Serial.print(F("  Cal:    S:"));
        Serial.print(c.system); Serial.print(F(" G:"));
        Serial.print(c.gyro); Serial.print(F(" A:"));
        Serial.print(c.accel); Serial.print(F(" M:"));
        Serial.println(c.mag);
    } else {
        Serial.print(F("FAULT 0x"));
        Serial.println(_imu ? _imu->getLastFault() : 0xFF, HEX);
    }

    Serial.print(F("Servos: "));
    if (_servo != nullptr && _servo->isHealthy()) {
        Serial.println(F("OK"));
        ServoState s = _servo->getState();
        for (int i = 0; i < 4; i++) {
            Serial.print(F("  Fin ")); Serial.print(i);
            Serial.print(F(":  ")); Serial.print(s.angle[i], 1);
            float t = _servo->getTrim(i);
            if (t != 0.0f) {
                Serial.print(F(" (trim "));
                if (t > 0) Serial.print('+');
                Serial.print(t, 1);
                Serial.print(F(")"));
            }
            Serial.println();
        }
    } else {
        Serial.println(F("NOT AVAILABLE"));
    }

    Serial.print(F("Guide:  "));
    if (_guide != nullptr) {
        Serial.print(_guide->isEnabled() ? F("ENABLED") : F("DISABLED"));
        Serial.print(F("  max_defl: "));
        Serial.print(_guide->getMaxDeflection(), 0);
        Serial.println(F(" deg"));
        if (_guide->isEnabled()) {
            Serial.print(F("  PID:    P:"));
            Serial.print(_guide->getPitchCmd(), 3);
            Serial.print(F("  Y:")); Serial.print(_guide->getYawCmd(), 3);
            Serial.print(F("  R:")); Serial.println(_guide->getRollCmd(), 3);
        }
    } else {
        Serial.println(F("N/A"));
    }

    Serial.println(F("---------------"));
}

void Commands::cmdCal() {
    if (_imu == nullptr || !_imu->isHealthy()) {
        Serial.println(F("ERR: IMU not healthy"));
        return;
    }
    CalibrationStatus c = _imu->getCalibration();
    Serial.println(F("--- BNO055 CALIBRATION ---"));
    Serial.print(F("System: ")); Serial.print(c.system); Serial.println(F("/3"));
    Serial.print(F("Gyro:   ")); Serial.print(c.gyro);   Serial.println(F("/3"));
    Serial.print(F("Accel:  ")); Serial.print(c.accel);  Serial.println(F("/3"));
    Serial.print(F("Mag:    ")); Serial.print(c.mag);    Serial.println(F("/3"));
    bool ok = (c.system == 3 && c.gyro == 3 && c.accel == 3 && c.mag == 3);
    Serial.println(ok ? F("Fully calibrated.") : F("Move rocket through all orientations."));
    Serial.println(F("--------------------------"));
}

void Commands::cmdServo(const char* arg) {
    if (_servo == nullptr || !_servo->isHealthy()) {
        Serial.println(F("ERR: servo driver not available"));
        return;
    }
    if (arg == nullptr || *arg == '\0') {
        ServoState s = _servo->getState();
        Serial.println(F("--- SERVOS ---"));
        for (int i = 0; i < 4; i++) {
            Serial.print(F("  Fin ")); Serial.print(i);
            Serial.print(F(": ")); Serial.print(s.angle[i], 1);
            float t = _servo->getTrim(i);
            if (t != 0.0f) {
                Serial.print(F(" (trim "));
                if (t > 0) Serial.print('+');
                Serial.print(t, 1);
                Serial.print(F(")"));
            }
            Serial.println(F(" deg"));
        }
        Serial.println(F("--------------"));
        return;
    }
    int idx = atoi(arg);
    char* second = strchr(arg, ' ');
    if (second == nullptr || idx < 0 || idx > 3) {
        Serial.println(F("usage: SERVO <0-3> <angle>"));
        return;
    }
    float angle = atof(second + 1);
    if (angle < 0.0f || angle > 180.0f) {
        Serial.println(F("ERR: angle must be 0-180"));
        return;
    }
    if (_guide != nullptr && _guide->isEnabled()) {
        Serial.println(F("ERR: disable guidance first (GUIDE off)"));
        return;
    }
    _servo->setAngle(idx, angle);
    Serial.print(F("Fin ")); Serial.print(idx);
    Serial.print(F(" -> ")); Serial.print(angle, 1);
    Serial.println(F(" deg"));
}

void Commands::cmdCenter() {
    if (_servo == nullptr || !_servo->isHealthy()) {
        Serial.println(F("ERR: servo driver not available"));
        return;
    }
    if (_guide != nullptr && _guide->isEnabled()) {
        Serial.println(F("ERR: disable guidance first (GUIDE off)"));
        return;
    }
    _servo->center();
    Serial.println(F("All fins centered (90 deg)"));
}

void Commands::cmdTrim(const char* arg) {
    if (_servo == nullptr || !_servo->isHealthy()) {
        Serial.println(F("ERR: servo driver not available"));
        return;
    }
    if (arg == nullptr || *arg == '\0') {
        Serial.println(F("--- TRIM ---"));
        for (int i = 0; i < 4; i++) {
            float t = _servo->getTrim(i);
            Serial.print(F("  Fin ")); Serial.print(i); Serial.print(F(": "));
            if (t >= 0) Serial.print('+');
            Serial.print(t, 1); Serial.println(F(" deg"));
        }
        Serial.println(F("  TRIM <0-3> <offset>  adjust"));
        Serial.println(F("  TRIM CLEAR           zero all"));
        Serial.println(F("------------"));
        return;
    }
    if (strcasecmp(arg, "CLEAR") == 0) {
        _servo->clearAllTrim();
        Serial.println(F("All trim cleared"));
        return;
    }
    int idx = atoi(arg);
    char* second = strchr(arg, ' ');
    if (second == nullptr || idx < 0 || idx > 3) {
        Serial.println(F("usage: TRIM <0-3> <offset>"));
        return;
    }
    float offset = atof(second + 1);
    if (offset < -15.0f || offset > 15.0f) {
        Serial.println(F("ERR: trim range is -15 to +15 deg"));
        return;
    }
    _servo->setTrim(idx, offset);
    Serial.print(F("Fin ")); Serial.print(idx);
    Serial.print(F(" trim -> "));
    if (offset >= 0) Serial.print('+');
    Serial.print(offset, 1); Serial.println(F(" deg"));
}

void Commands::cmdGuide(const char* arg) {
    if (_guide == nullptr) {
        Serial.println(F("ERR: guidance not available"));
        return;
    }
    if (arg == nullptr || *arg == '\0') {
        Serial.println(F("--- GUIDANCE ---"));
        Serial.print(F("State:    "));
        Serial.println(_guide->isEnabled() ? F("ENABLED") : F("DISABLED"));
        Serial.print(F("Max defl: "));
        Serial.print(_guide->getMaxDeflection(), 1);
        Serial.println(F(" deg"));
        if (_guide->isEnabled()) {
            Serial.print(F("Pitch:    ")); Serial.println(_guide->getPitchCmd(), 3);
            Serial.print(F("Yaw:      ")); Serial.println(_guide->getYawCmd(), 3);
            Serial.print(F("Roll:     ")); Serial.println(_guide->getRollCmd(), 3);
        }
        Serial.println(F("----------------"));
        return;
    }
    if (strcasecmp(arg, "on") == 0) {
        if (_imu == nullptr || !_imu->isHealthy()) {
            Serial.println(F("ERR: IMU not healthy"));
            return;
        }
        if (_servo == nullptr || !_servo->isHealthy()) {
            Serial.println(F("ERR: servos not healthy"));
            return;
        }
        _guide->enable();
        Serial.println(F("Guidance ENABLED — hold rocket in target orientation"));
    } else if (strcasecmp(arg, "off") == 0) {
        _guide->disable();
        Serial.println(F("Guidance DISABLED, fins centered"));
    } else {
        Serial.println(F("usage: GUIDE on|off"));
    }
}

void Commands::cmdGains(const char* arg) {
    if (_guide == nullptr) {
        Serial.println(F("ERR: guidance not available"));
        return;
    }
    if (arg == nullptr || *arg == '\0') {
        Serial.println(F("usage: GAINS <p|y|r> <kp> <ki> <kd>"));
        Serial.println(F("  e.g. GAINS p 0.5 0.0 0.1"));
        return;
    }
    char axis = tolower(arg[0]);
    if (axis != 'p' && axis != 'y' && axis != 'r') {
        Serial.println(F("ERR: axis must be p, y, or r"));
        return;
    }
    char* ptr = (char*)(arg + 1);
    while (*ptr == ' ') ptr++;
    float kp = strtof(ptr, &ptr);
    while (*ptr == ' ') ptr++;
    float ki = strtof(ptr, &ptr);
    while (*ptr == ' ') ptr++;
    float kd = strtof(ptr, &ptr);

    if (kp < 0 || ki < 0 || kd < 0) {
        Serial.println(F("ERR: gains must be >= 0"));
        return;
    }
    switch (axis) {
        case 'p': _guide->setPitchGains(kp, ki, kd); Serial.print(F("Pitch: ")); break;
        case 'y': _guide->setYawGains(kp, ki, kd);   Serial.print(F("Yaw:   ")); break;
        case 'r': _guide->setRollGains(kp, ki, kd);   Serial.print(F("Roll:  ")); break;
    }
    Serial.print(F("Kp=")); Serial.print(kp, 3);
    Serial.print(F(" Ki=")); Serial.print(ki, 3);
    Serial.print(F(" Kd=")); Serial.println(kd, 3);
}

void Commands::cmdArm() {
    if (_fram->GetErrorCodeByte() != 0x00) {
        Serial.println(F("ERR: cannot ARM with active fault"));
        return;
    }
    if (_fram->GetProgramState() != static_cast<uint8_t>(ProgramState::IDLE)) {
        Serial.println(F("ERR: ARM only allowed from IDLE"));
        return;
    }
    if (_imu != nullptr && !_imu->isHealthy()) {
        Serial.println(F("ERR: IMU not healthy"));
        return;
    }
    if (_servo != nullptr && !_servo->isHealthy()) {
        Serial.println(F("ERR: servos not healthy"));
        return;
    }
    _fram->SetProgramState(static_cast<uint8_t>(ProgramState::ARMED));
    Serial.println(F("ARMED. Watching for launch."));
}

void Commands::cmdDisarm() {
    if (_fram->GetProgramState() != static_cast<uint8_t>(ProgramState::ARMED)) {
        Serial.println(F("ERR: DISARM only allowed from ARMED"));
        return;
    }
    if (_guide != nullptr && _guide->isEnabled()) {
        _guide->disable();
    }
    _fram->SetProgramState(static_cast<uint8_t>(ProgramState::IDLE));
    Serial.println(F("Disarmed. Returning to IDLE."));
}


void Commands::cmdMonitor(const char* arg) {
    if (arg == nullptr || *arg == '\0') {
        Serial.println(F("usage: MON <sensors|servos|pid|guide|all> [ms]"));
        return;
    }

    // Parse mode
    char modeBuf[16];
    const char* space = strchr(arg, ' ');
    size_t len = space ? (size_t)(space - arg) : strlen(arg);
    if (len >= sizeof(modeBuf)) len = sizeof(modeBuf) - 1;
    memcpy(modeBuf, arg, len);
    modeBuf[len] = '\0';

    MonitorMode mode = MonitorMode::NONE;
    if      (strcasecmp(modeBuf, "sensors") == 0) mode = MonitorMode::SENSORS;
    else if (strcasecmp(modeBuf, "servos")  == 0) mode = MonitorMode::SERVOS;
    else if (strcasecmp(modeBuf, "pid")     == 0) mode = MonitorMode::PID;
    else if (strcasecmp(modeBuf, "guide")   == 0) mode = MonitorMode::GUIDE;
    else if (strcasecmp(modeBuf, "all")     == 0) mode = MonitorMode::ALL;
    else if (strcasecmp(modeBuf, "stop")    == 0) {
        monStop();
        return;
    }
    else {
        Serial.println(F("ERR: unknown mode (sensors|servos|pid|guide|all)"));
        return;
    }
    if (space != nullptr) {
        unsigned long ms = atol(space + 1);
        if (ms >= 50 && ms <= 5000) {
            _monIntervalMs = ms;
        } else {
            Serial.println(F("ERR: interval must be 50-5000 ms"));
            return;
        }
    }

    _monMode = mode;
    _monLastMs = millis();
    Serial.print(F("Monitoring ")); Serial.print(modeBuf);
    Serial.print(F(" @ ")); Serial.print(_monIntervalMs);
    Serial.println(F("ms (any key to stop)"));
    Serial.println();
}

void Commands::cmdUnknown(const char* verb) {
    Serial.print(F("unknown command: "));
    Serial.println(verb);
    Serial.println(F("type HELP for available commands"));
}
