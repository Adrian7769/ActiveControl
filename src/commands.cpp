#include "commands.h"
#include <string.h>
#include <ctype.h>
#include "config.h"
Commands::Commands(FRAM* fram)
    : _fram(fram), _pos(0), _echo(true)
{
    _buffer[0] = '\0';
}

void Commands::begin() {
    printWelcome();
    printPrompt();
}

void Commands::tick() {
    if (readLine()) {
        dispatch(_buffer);
        resetBuffer();
        printPrompt();
    }
}

// ---------- Reader ----------

bool Commands::readLine() {
    while (Serial.available()) {
        char c = Serial.read();

        // Ignore carriage returns — handles Windows \r\n
        if (c == '\r') continue;

        // Newline = end of command
        if (c == '\n') {
            if (_echo) Serial.println();
            _buffer[_pos] = '\0';
            return _pos > 0;          // ignore empty lines
        }

        // Backspace or DEL
        if (c == '\b' || c == 127) {
            if (_pos > 0) {
                _pos--;
                if (_echo) Serial.print("\b \b");
            }
            continue;
        }

        // Printable character
        if (_pos < CMD_BUFFER_SIZE - 1) {
            _buffer[_pos++] = c;
            if (_echo) Serial.write(c);
        }
        // else: buffer full, drop silently until newline
    }
    return false;
}

void Commands::resetBuffer() {
    _pos = 0;
    _buffer[0] = '\0';
}

// ---------- Dispatch ----------

void Commands::dispatch(char* line) {
    // Split into verb + remaining args
    char* verb = strtok(line, " \t");
    char* arg  = strtok(nullptr, "");      // everything after the first space

    if (verb == nullptr) return;

    // Trim leading whitespace from arg if present
    while (arg && *arg == ' ') arg++;

    if      (strcasecmp(verb, "STATUS")      == 0) cmdStatus();
    else if (strcasecmp(verb, "HELP")        == 0) cmdHelp();
    else if (strcasecmp(verb, "?")           == 0) cmdHelp();
    else if (strcasecmp(verb, "DUMP")        == 0) cmdDump();
    else if (strcasecmp(verb, "CLEAR")       == 0) cmdClear();
    else if (strcasecmp(verb, "ARM")         == 0) cmdArm();
    else if (strcasecmp(verb, "DISARM")      == 0) cmdDisarm();
    else if (strcasecmp(verb, "CLEAR_FAULT") == 0) cmdClearFault();
    else if (strcasecmp(verb, "ECHO")        == 0) cmdEcho(arg);
    else                                           cmdUnknown(verb);
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
        case ProgramState::UNINITIALIZED: return "UNINIT";
        case ProgramState::IDLE:          return "IDLE";
        case ProgramState::FAULT:         return "FAULT";
        case ProgramState::ARMED:         return "ARMED";
        case ProgramState::ASCENT:        return "ASCENT";
        case ProgramState::APOGEE:        return "APOGEE";
        case ProgramState::DESCENT:       return "DESCENT";
        case ProgramState::LANDED:        return "LANDED";
        case ProgramState::POST_FLIGHT:   return "POST_FLIGHT";
        default:                          return "???";
    }
}

// ---------- Command handlers ----------

void Commands::cmdHelp() {
    Serial.println(F("Available commands:"));
    Serial.println(F("  STATUS         - show current state and counters"));
    Serial.println(F("  HELP / ?       - this list"));
    Serial.println(F("  DUMP           - dump all FRAM records"));
    Serial.println(F("  CLEAR          - erase all records (POST_FLIGHT only)"));
    Serial.println(F("  ARM            - IDLE -> ARMED"));
    Serial.println(F("  DISARM         - ARMED -> IDLE"));
    Serial.println(F("  CLEAR_FAULT    - FAULT -> IDLE"));
    Serial.println(F("  ECHO on|off    - toggle character echo"));
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
    _fram->DumpControlBlock();
    _fram->DumpDataBytes();
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

void Commands::cmdClearFault() {
    if (_fram->GetProgramState() != static_cast<uint8_t>(ProgramState::FAULT)) {
        Serial.println(F("ERR: CLEAR_FAULT only allowed from FAULT"));
        return;
    }
    _fram->SetErrorCodeByte(0);
    _fram->SetProgramState(static_cast<uint8_t>(ProgramState::IDLE));
    Serial.println(F("Fault cleared. Returning to IDLE."));
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
