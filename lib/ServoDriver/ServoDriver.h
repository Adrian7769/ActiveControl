#ifndef SERVODRIVER_H
#define SERVODRIVER_H

#include <Adafruit_PWMServoDriver.h>

struct ServoState {
    float angle[4];  // current angle 0-180 for each servo
};

class ServoDriver {
public:
    ServoDriver();
    uint8_t begin();
    void tick();

    // Set a single servo angle (0.0 – 180.0)
    void setAngle(uint8_t index, float degrees);

    // Set all four simultaneously
    void setAll(float d0, float d1, float d2, float d3);

    // Center all servos to 90 degrees
    void center();

    ServoState getState() const { return _state; }
    bool    isHealthy()    const { return _healthy; }
    uint8_t getLastFault() const { return _last_fault; }

private:
    Adafruit_PWMServoDriver _pca;
    ServoState _state;
    bool    _healthy;
    uint8_t _last_fault;

    // Maps channel index (0-3) to actual PCA9685 channel
    uint8_t _channel_map[4];

    uint16_t angleToPulse(float degrees);
};

#endif
