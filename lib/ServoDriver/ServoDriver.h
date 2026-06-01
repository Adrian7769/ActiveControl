#ifndef SERVODRIVER_H
#define SERVODRIVER_H

#include <Arduino.h>
#include <ESP32Servo.h>

static constexpr uint8_t SERVO_COUNT = 4;

struct ServoState {
    float angle[SERVO_COUNT];
};

class ServoDriver {
public:
    ServoDriver();
    uint8_t begin();
    void tick();

    void setAngle(uint8_t index, float degrees);
    void setAll(float d0, float d1, float d2, float d3);
    void center();

    void  setTrim(uint8_t index, float degrees);
    float getTrim(uint8_t index) const;
    void  clearAllTrim();

    ServoState getState() const { return _state; }
    bool    isHealthy()    const { return _healthy; }
    uint8_t getLastFault() const { return _last_fault; }

private:
    Servo _servo[SERVO_COUNT];
    ServoState _state;
    float   _trim[SERVO_COUNT];   // per-servo trim offset in degrees
    bool    _healthy;
    uint8_t _last_fault;

    static constexpr uint8_t SERVO_PINS[SERVO_COUNT] = { 25, 26, 27, 14 };
    static constexpr int PULSE_MIN_US = 1000;  // 0 degrees
    static constexpr int PULSE_MAX_US = 2000;  // 180 degrees
    static constexpr int PWM_HZ       = 50;
};

#endif
