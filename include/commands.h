#ifndef COMMANDS_H
#define COMMANDS_H

#include <Arduino.h>
#include "fram.h"

class Commands {
public:
    // Constructor Commands needs a FRAM to operate on
    Commands(FRAM* fram);

    // Call once from setup() after Fram.begin()
    void begin();

    // Call every iteration of loop()
    void tick();

    // Runtime setting
    void setEcho(bool on) { _echo = on; }
    bool getEcho() const  { return _echo; }

private:
    // Dependencies (not owned)
    FRAM* _fram;

    // Reader state
    static constexpr size_t CMD_BUFFER_SIZE = 64;
    char   _buffer[CMD_BUFFER_SIZE];
    size_t _pos;
    bool   _echo;

    // Reader / parser / dispatcher
    bool readLine();
    void resetBuffer();
    void dispatch(char* line);

    // Display helpers
    void printWelcome();
    void printPrompt();
    const char* stateName(ProgramState s);

    // Command handlers
    void cmdHelp();
    void cmdStatus();
    void cmdDump();
    void cmdClear();
    void cmdArm();
    void cmdDisarm();
    void cmdClearFault();
    void cmdEcho(const char* arg);
    void cmdUnknown(const char* verb);
};

#endif
