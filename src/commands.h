#ifndef COMMANDS_H
#define COMMANDS_H

#include <Arduino.h>
#include "fram.h"
#include "faults.h"
#include <Barometer.h>
#include <IMU.h>
#include "ServoDriver.h"
#include "Guidance.h"

enum class MonitorMode : uint8_t {
    NONE = 0,
    SENSORS,
    SERVOS,
    PID,
    GUIDE,
    ALL
};

class Commands {
public:
    Commands(FRAM* fram, IMU* Imu, Barometer* Baro, ServoDriver* servo, Guidance* guide);
    void begin();
    void tick();
    // Runtime setting
    void setEcho(bool on) { _echo = on; }
    bool getEcho() const  { return _echo; }

private:
    // Dependencies (not owned)
    FRAM* _fram;
    IMU* _imu;
    Barometer* _baro;
    ServoDriver* _servo;
    Guidance* _guide;
    // Reader state
    static constexpr size_t CMD_BUFFER_SIZE = 64;
    char   _buffer[CMD_BUFFER_SIZE];
    size_t _pos;
    bool   _echo;
    // Monitor state
    MonitorMode   _monMode;
    unsigned long _monLastMs;
    unsigned long _monIntervalMs;
    // Reader / parser / dispatcher
    bool readLine();
    void resetBuffer();
    void dispatch(char* line);
    // Display helpers
    void printWelcome();
    void printPrompt();
    const char* stateName(ProgramState s);
    // Monitor helpers
    void monTick();
    void monPrintSensors();
    void monPrintServos();
    void monPrintPID();
    void monPrintGuide();
    void monStop();
    // Command handlers System
    void cmdHelp();
    void cmdHelpFaults();
    void cmdStatus();
    void cmdDump(const char* arg);
    void cmdClear(const char* arg);
    void cmdEcho(const char* arg);
    // Command handlers Sensors
    void cmdSensors();
    void cmdCal();
    // Command handlers Actuators
    void cmdServo(const char* arg);
    void cmdCenter();
    void cmdTrim(const char* arg);
    // Command handlers Guidance
    void cmdGuide(const char* arg);
    void cmdGains(const char* arg);
    // Command handlers Flight
    void cmdArm();
    void cmdDisarm();
    // Command handlers Monitoring
    void cmdMonitor(const char* arg);

    void cmdUnknown(const char* verb);
};

#endif
