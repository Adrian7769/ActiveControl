#include "commands.h"
#include <string.h>
#include <ctype.h>
#include "config.h"
#include "faults.h"

Commands::Commands(FRAM* fram, IMU* Imu, Barometer* Baro) {
    _fram = fram;
    _baro = Baro;
    _imu = Imu;
    _pos = 0;
    _echo = true;
    _buffer[0] = '\0';
}

void Commands::begin() {
    printWelcome();
    printPrompt();
}

void Commands::tick() {
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
	// Enter
        if (c == '\n') { // enter
            if (_echo) {
		    Serial.println(); // print new line
	    }
            _buffer[_pos] = '\0'; // enter null character at _pos
            return true; 
        }
        if (c == '\b') {
            if (_pos > 0) {
                _pos--;
                if (_echo) { 
			Serial.print("\b \b"); // erase character effect
		}
            }
            continue;
        }
        if (_pos < CMD_BUFFER_SIZE - 1) {
            _buffer[_pos++] = c; // equivalent to: _buffer[_pos] = c; _pos = _pos + 1;
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

    if (strcasecmp(verb, "STATUS") == 0) {
	    cmdStatus();
    }
    else if (strcasecmp(verb, "HELP")   == 0) {
        if (arg != nullptr && strcasecmp(arg, "FAULT") == 0) {
		cmdHelpFaults();
	}
        else cmdHelp();
    }
    else if (strcasecmp(verb, "SENSORS") == 0) cmdSensors();
    else if (strcasecmp(verb, "CALIBRATE")     == 0) cmdCal();   
    else if (strcasecmp(verb, "?")      == 0) cmdHelp();
    else if (strcasecmp(verb, "DUMP")   == 0) cmdDump();
    else if (strcasecmp(verb, "CLEAR")  == 0) cmdClear();
    else if (strcasecmp(verb, "ARM")    == 0) cmdArm();
    else if (strcasecmp(verb, "DISARM") == 0) cmdDisarm();
    else if (strcasecmp(verb, "ECHO")   == 0) cmdEcho(arg);
    else cmdUnknown(verb);
}
// ---------- Display helpers ----------

void Commands::printWelcome() {
    Serial.println();
    Serial.println(F("================================"));
    Serial.print(F("  Flight Computer  v"));Serial.println(VERSION);
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
        case ProgramState::UNINITIALIZED: 
		return "UNINIT";
        case ProgramState::IDLE:          
		return "IDLE";
        case ProgramState::FAULT:         
		return "FAULT";
        case ProgramState::ARMED:         
		return "ARMED";
        case ProgramState::ASCENT:        
		return "ASCENT";
        case ProgramState::APOGEE:        
		return "APOGEE";
        case ProgramState::DESCENT:       
		return "DESCENT";
        case ProgramState::LANDED:        
		return "LANDED";
        case ProgramState::POST_FLIGHT:   
		return "POST_FLIGHT";
        default:                          
		return "N/A";
    }
}
	
// ---------- Command handlers ----------

void Commands::cmdCal() {
    if (_imu == nullptr) {
        Serial.println(F("ERR: imu pointer not wired"));
        return;
    }
    if (!_imu->isHealthy()) {
        Serial.println(F("ERR: IMU not healthy"));
        return;
    }
    
    CalibrationStatus c = _imu->getCalibration();
    Serial.println(F("--- BNO055 CALIBRATION ---"));
    Serial.print(F("System: ")); Serial.print(c.system); Serial.println(F("/3"));
    Serial.print(F("Gyro:   ")); Serial.print(c.gyro);   Serial.println(F("/3"));
    Serial.print(F("Accel: "));  Serial.print(c.accel);  Serial.println(F("/3"));
    Serial.print(F("Mag:    ")); Serial.print(c.mag);    Serial.println(F("/3"));
    Serial.println(F("Move the rocket through all orientations to calibrate."));
    Serial.println(F("--------------------------"));
}

void Commands::cmdSensors() {
    if (_baro == nullptr || _imu == nullptr) {
        Serial.println(F("ERR: sensor pointers not wired"));
        return;
    }
    
    Serial.println(F("--- SENSORS ---"));
    Serial.print(F("Baro: "));
    if (_baro->isHealthy()) {
        Serial.print(_baro->getPressurePa(), 1); Serial.print(F(" Pa, "));
        Serial.print(_baro->getTemperatureC(), 2); Serial.print(F(" C, "));
        Serial.print(_baro->getAltitudeM(), 2); Serial.println(F(" m"));
    } else {
        Serial.print(F("FAULT 0x")); Serial.println(_baro->getLastFault(), HEX);
    }
    
    Serial.print(F("IMU:  "));
    if (_imu->isHealthy()) {
        Orientation o = _imu->getOrientation();
        Quaternion  q = _imu->getQuaternion();
        Vec3        a = _imu->getLinearAccel();
        Vec3        g = _imu->getGravityVector();
        Serial.print(F("Eul: ")); 
        Serial.print(o.heading, 1); Serial.print(F(" "));
        Serial.print(o.roll, 1);    Serial.print(F(" "));
        Serial.println(o.pitch, 1);
        Serial.print(F("Qua: "));
        Serial.print(q.w, 3); Serial.print(F(" "));
        Serial.print(q.x, 3); Serial.print(F(" "));
        Serial.print(q.y, 3); Serial.print(F(" "));
        Serial.println(q.z, 3);
        Serial.print(F("LinAcc: "));
        Serial.print(a.x, 1); Serial.print(F(","));
        Serial.print(a.y, 1); Serial.print(F(","));
        Serial.println(a.z, 1);
        Serial.print(F("Grav: "));
        Serial.print(g.x, 1); Serial.print(F(","));
        Serial.print(g.y, 1); Serial.print(F(","));
        Serial.println(g.z, 1);;
    } else {
        Serial.print(F("FAULT 0x")); Serial.println(_imu->getLastFault(), HEX);
    }
    Serial.println(F("---------------"));
}

void Commands::cmdHelp() {
    Serial.println(F("Available commands:"));
    Serial.println(F("  STATUS         - show current state and counters"));
    Serial.println(F("  HELP / ?       - this list"));
    Serial.println(F("  HELP FAULT     - list fault codes"));
    Serial.println(F("  SENSORS        - sensor test"));
    Serial.println(F("  CALIBRATE      - calibrate BNO055"));
    Serial.println(F("  DUMP           - dump all FRAM records"));
    Serial.println(F("  CLEAR          - erase all records (POST_FLIGHT only)"));
    Serial.println(F("  ARM            - IDLE -> ARMED"));
    Serial.println(F("  DISARM         - ARMED -> IDLE"));
    Serial.println(F("  ECHO on|off    - toggle character echo"));
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
        
        // Left-pad the name to 20 chars for alignment
        Serial.print(f.name);
        for (size_t pad = strlen(f.name); pad < 20; ++pad) Serial.print(' ');
        
        Serial.print(F("  "));
        Serial.println(f.description);
    }
    
    Serial.println(F("-------------------"));
    
    // Show current error byte for context
    uint8_t current = _fram->GetErrorCodeByte();
    Serial.print(F("Current error code: 0x"));
    if (current < 0x10) Serial.print('0');
    Serial.print(current, HEX);
    
    const FaultEntry* entry = lookupFault(current);
    if (entry) {
        Serial.print(F("  ("));
        Serial.print(entry->name);
        Serial.println(F(")"));
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
    Serial.println(F("--------------"));
}

void Commands::cmdDump() {
    Serial.println(F("Dumping all records:"));
    _fram->DumpControlBlock(); // Control Block
    _fram->DumpDataBytes(); // All datablocks
    Serial.println(F("Dump complete."));
}

void Commands::cmdClear() {
    uint8_t s = _fram->GetProgramState();
    if (s != static_cast<uint8_t>(ProgramState::POST_FLIGHT) &&
        s != static_cast<uint8_t>(ProgramState::FAULT)) {
        Serial.println(F("ERR: CLEAR only allowed in POST_FLIGHT or FAULT"));
        return;
    }
    Serial.println(F("Clearing FRAM..."));
    //_fram->ResetFram();
    Serial.println(F("Cleared. Moving to IDLE."));
    _fram->SetProgramState(static_cast<uint8_t>(ProgramState::IDLE));
}

void Commands::cmdArm() {
    if (_fram->GetErrorCodeByte() != 0x00) {
	Serial.println(F("ERR: Unable to ARM from FAULT."));
	return;
    }
    if (_fram->GetProgramState() != static_cast<uint8_t>(ProgramState::IDLE)) {
        Serial.println(F("ERR: ARM only allowed from IDLE"));
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
    _fram->SetProgramState(static_cast<uint8_t>(ProgramState::IDLE));
    Serial.println(F("Disarmed. Returning to IDLE."));
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

void Commands::cmdUnknown(const char* verb) {
    Serial.print(F("unknown command: "));
    Serial.println(verb);
    Serial.println(F("type HELP for available commands"));
}
