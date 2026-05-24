#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "fram.h"
#include "commands.h"

// ---- Globals ----
FRAM Fram;                 // one and only FRAM instance
Commands Cmd(&Fram);           // Commands gets a pointer to Fram
DataBlock Block;

// Periodic write timer (test scaffolding — will be replaced by Telemetry later)
unsigned long lastRun = 0;
const unsigned long writeInterval = 10000;   // 10 s

void setup() {
    Serial.begin(115200);
    delay(2000);                // give USB time to settle
    Fram.begin();
    // Post
    uint8_t prev_state = Fram.GetProgramState(); // 0x00 for uninit
    bool postOk = true;
    if (prev_state == static_cast<uint8_t>(ProgramState::UNINITIALIZED)) {
	    Fram.SetProgramState(postOk ? static_cast<uint8_t>(ProgramState::IDLE) : static_cast<uint8_t>(ProgramState::FAULT));
    } else if (prev_state >= static_cast<uint8_t>(ProgramState::ARMED) && prev_state <= static_cast<uint8_t>(ProgramState::POST_FLIGHT)) {
    	    Fram.SetProgramState(static_cast<uint8_t>(ProgramState::POST_FLIGHT));
    }
    Cmd.begin();	    
}

void loop() {
    // Service commands every iteration
    Cmd.tick();
    if ((millis() - lastRun) >= writeInterval) {
        lastRun = millis();
        Block.timestamp_ms = millis();
        Fram.WriteDataBlock(&Block);
    }
}
