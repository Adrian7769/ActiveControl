#ifndef COMMANDS_H
#define COMMANDS_H

#include <Arduino.h>
#include "fram.h"
#include "faults.h"
#include <Barometer.h>
#include <IMU.h>
#include <ServoDriver.h>

class Commands {
public:
    Commands(FRAM* fram, IMU* Imu, Barometer* Baro, ServoDriver* Servos);
    void begin();
    void tick();
    // Runtime setting
    void setEcho(bool on) { _echo = on; }
    bool getEcho() const  { return _echo; }

private:
    // Dependencies
    FRAM* _fram;
    IMU* _imu;
    Barometer* _baro;
    ServoDriver* _servos;


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
    const char* stateName(ProgramState s);  // Fetch State char * for Given ProgramState value
    // Command handlers
    void cmdHelp();
    void cmdHelpFaults();
    void cmdSensors();
    void cmdCal();
    void cmdStatus();
    void cmdDump();
    void cmdClear();
    void cmdArm();
    void cmdDisarm();
    void cmdEcho(const char* arg);
    void cmdUnknown(const char* verb);
};

#endif
