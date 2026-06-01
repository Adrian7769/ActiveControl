#include "Guidance.h"
#include <Arduino.h>
#include <math.h>

Guidance::Guidance(IMU* imu, ServoDriver* servos)
    : _imu(imu), _servos(servos)
    , _pid_pitch(0, 0, 0, -1, 1)
    , _pid_yaw(0, 0, 0, -1, 1)
    , _pid_roll(0, 0, 0, -1, 1)
    , _max_defl(15.0f)
    , _enabled(false)
    , _last_tick_us(0)
    , _pitch_cmd(0), _yaw_cmd(0), _roll_cmd(0)
{}

void Guidance::begin(const GuidanceConfig& cfg) {
    _pid_pitch = PIDController(cfg.kp_pitch, cfg.ki_pitch, cfg.kd_pitch, -1, 1);
    _pid_yaw   = PIDController(cfg.kp_yaw,   cfg.ki_yaw,   cfg.kd_yaw,  -1, 1);
    _pid_roll  = PIDController(cfg.kp_roll,  cfg.ki_roll,  cfg.kd_roll,  -1, 1);
    _max_defl  = cfg.max_deflection;
}

void Guidance::tick() {
    if (!_enabled) return;
    if (!_imu->isHealthy() || !_servos->isHealthy()) return;

    unsigned long now = micros();
    unsigned long elapsed = now - _last_tick_us;
    if (elapsed < TICK_INTERVAL_US) return;

    float dt = elapsed / 1e6f;
    _last_tick_us = now;

    if (dt > 0.05f) dt = 0.05f;

    Quaternion q = _imu->getQuaternion();

    float pitch_err, yaw_err, roll_err;
    computeErrors(q, pitch_err, yaw_err, roll_err);

    _pitch_cmd = _pid_pitch.compute(pitch_err, dt);
    _yaw_cmd   = _pid_yaw.compute(yaw_err, dt);
    _roll_cmd  = _pid_roll.compute(roll_err, dt);

    mixFins(_pitch_cmd, _yaw_cmd, _roll_cmd);
}

void Guidance::computeErrors(const Quaternion& q,
                              float& pitch_err,
                              float& yaw_err,
                              float& roll_err)
{
    // Error = q_ref * inverse(q_current)
    // This gives the rotation FROM current TO reference.
    // inverse(q_current) = conjugate for unit quaternions
    // q_error = q_ref * conj(q_current)
    // Quaternion multiplication:
    // (a.w + a.xi + a.yj + a.zk)(b.w + b.xi + b.yj + b.zk)

    float cw =  q.w;   // conjugate of current
    float cx = -q.x;
    float cy = -q.y;
    float cz = -q.z;

    // q_error = q_ref * conjugate(q_current)
    float ew = _q_ref.w*cw - _q_ref.x*cx - _q_ref.y*cy - _q_ref.z*cz;
    float ex = _q_ref.w*cx + _q_ref.x*cw + _q_ref.y*cz - _q_ref.z*cy;
    float ey = _q_ref.w*cy - _q_ref.x*cz + _q_ref.y*cw + _q_ref.z*cx;
    float ez = _q_ref.w*cz + _q_ref.x*cy - _q_ref.y*cx + _q_ref.z*cw;
    // thbis should ensure shortes rotation path
    if (ew < 0) { ew = -ew; ex = -ex; ey = -ey; ez = -ez; }
    constexpr float RAD2DEG = 180.0f / PI;
    pitch_err = 2.0f * ex * RAD2DEG;
    yaw_err   = 2.0f * ey * RAD2DEG;
    roll_err  = 2.0f * ez * RAD2DEG;
}

void Guidance::mixFins(float pitch_cmd, float yaw_cmd, float roll_cmd) {
    float p = pitch_cmd * _max_defl;
    float y = yaw_cmd   * _max_defl;
    float r = roll_cmd  * _max_defl;

    //         Fin 0
    //           |
    //  Fin 3 ---+--- Fin 1
    //           |
    //         Fin 2
    // Fin 0 (top)    and Fin 2 (bottom) control PITCH only.
    // Fin 1 (right)  and Fin 3 (left)   control YAW only.
    // Roll: all four fins deflect in the same rotational sense.

    float fin0 = 90.0f + p + r;   // top:    pitch
    float fin1 = 90.0f + y + r;   // right:  yaw
    float fin2 = 90.0f - p + r;   // bottom: pitch (opposite)
    float fin3 = 90.0f - y + r;   // left:   yaw (opposite)

    _servos->setAll(fin0, fin1, fin2, fin3);
}

void Guidance::enable() {
    _pid_pitch.reset();
    _pid_yaw.reset();
    _pid_roll.reset();
    // Capture current orientation as the target.
    // The rocket must be in its desired orientation (vertical)
    // when guidance is enabled.
    _q_ref = _imu->getQuaternion();
    _pitch_cmd = 0;
    _yaw_cmd   = 0;
    _roll_cmd  = 0;
    _last_tick_us = micros();
    _enabled = true;
}
void Guidance::disable() {
    _enabled = false;
    _pitch_cmd = 0;
    _yaw_cmd   = 0;
    _roll_cmd  = 0;
    _servos->center();
}

void Guidance::setPitchGains(float kp, float ki, float kd) {
    _pid_pitch.setGains(kp, ki, kd);
}

void Guidance::setYawGains(float kp, float ki, float kd) {
    _pid_yaw.setGains(kp, ki, kd);
}

void Guidance::setRollGains(float kp, float ki, float kd) {
    _pid_roll.setGains(kp, ki, kd);
}

void Guidance::setMaxDeflection(float deg) {
    if (deg < 1.0f)  deg = 1.0f;
    if (deg > 45.0f) deg = 45.0f;
    _max_defl = deg;
}
