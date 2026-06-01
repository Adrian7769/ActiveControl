#ifndef GUIDANCE_H
#define GUIDANCE_H

#include "PID.h"
#include "IMU.h"
#include "ServoDriver.h"

struct GuidanceConfig {
    float kp_pitch, ki_pitch, kd_pitch;
    float kp_yaw,   ki_yaw,   kd_yaw;
    float kp_roll,  ki_roll,  kd_roll;
    float max_deflection;   // max servo deflection from center (degrees)
};

static constexpr GuidanceConfig GUIDANCE_DEFAULTS = {
    0.5f, 0.0f, 0.1f,    // pitch
    0.5f, 0.0f, 0.1f,    // yaw
    0.3f, 0.0f, 0.05f,   // roll (less aggressive  roll is less critical)
    15.0f                 // max deflection degrees from center
};

class Guidance {
public:
    Guidance(IMU* imu, ServoDriver* servos);
    void begin(const GuidanceConfig& cfg = GUIDANCE_DEFAULTS);
    void tick();
    void enable();
    void disable();

    bool isEnabled() const { return _enabled; }

    float getPitchCmd() const { return _pitch_cmd; }
    float getYawCmd()   const { return _yaw_cmd; }
    float getRollCmd()  const { return _roll_cmd; }

    void setPitchGains(float kp, float ki, float kd);
    void setYawGains(float kp, float ki, float kd);
    void setRollGains(float kp, float ki, float kd);
    void setMaxDeflection(float deg);
    float getMaxDeflection() const { return _max_defl; }

private:
    IMU*         _imu;
    ServoDriver* _servos;

    PIDController _pid_pitch;
    PIDController _pid_yaw;
    PIDController _pid_roll;
    Quaternion _q_ref;
    float _max_defl;
    bool  _enabled;
    unsigned long _last_tick_us;
    float _pitch_cmd;
    float _yaw_cmd;
    float _roll_cmd;

    void computeErrors(const Quaternion& q,
                       float& pitch_err, float& yaw_err, float& roll_err);
    void mixFins(float pitch_cmd, float yaw_cmd, float roll_cmd);

    static constexpr unsigned long TICK_INTERVAL_US = 10000; // 100 Hz
};

#endif
