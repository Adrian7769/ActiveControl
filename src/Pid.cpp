#include "PID.h"

PID::PID(float kp, float ki, float kd, float out_min, float out_max)
    : _kp(kp), _ki(ki), _kd(kd)
    , _integral(0), _prev_error(0)
    , _out_min(out_min), _out_max(out_max)
    , _first(true)
{}

float PID::compute(float error, float dt_s) {
    if (dt_s <= 0) return 0;
    float p_term = _kp * error;
    _integral += error * dt_s;
    float i_term = _ki * _integral;
    if (_ki != 0) {
        if (i_term > _out_max) { _integral = _out_max / _ki; i_term = _out_max; }
        if (i_term < _out_min) { _integral = _out_min / _ki; i_term = _out_min; }
    }
    float d_term = 0;
    if (!_first) {
        d_term = _kd * (error - _prev_error) / dt_s;
    }
    _first = false;
    _prev_error = error;
    float out = p_term + i_term + d_term;
    if (out > _out_max) out = _out_max;
    if (out < _out_min) out = _out_min;
    return out;
}

void PID::reset() {
    _integral = 0;
    _prev_error = 0;
    _first = true;
}

void PID::setGains(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
    reset();
}
