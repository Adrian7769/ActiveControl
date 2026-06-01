#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
public:
    PIDController(float kp, float ki, float kd, float out_min, float out_max);
    float compute(float error, float dt_s);
    void reset();
    void setGains(float kp, float ki, float kd);
    float getKp() const { return _kp; }
    float getKi() const { return _ki; }
    float getKd() const { return _kd; }

private:
    float _kp, _ki, _kd;
    float _integral;
    float _prev_error;
    float _out_min, _out_max;
    bool  _first;
};

#endif
