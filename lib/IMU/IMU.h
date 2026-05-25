#ifndef IMU_H
#define IMU_H

#include <DFRobot_BNO055.h>

struct Orientation {
    float heading;  // degrees, 0-360
    float roll;     // degrees, -180 to +180
    float pitch;    // degrees, -90 to +90
};
struct Quaternion {
	float w, x, y, z;
};
struct Vec3 {
    float x, y, z;
};

struct CalibrationStatus {
    uint8_t system;  // 0-3, 3 = fully calibrated
    uint8_t gyro;
    uint8_t accel;
    uint8_t mag;
};

class IMU {
public:
    IMU();
    uint8_t begin();
    void tick();
    Quaternion getQuaternion() const { return _quaternion; }
    Vec3 getGravityVector() const { return _gravity; }    
    Orientation getOrientation() const { return _orientation; }
    Vec3 getLinearAccel()  const { return _linear_accel; }
    Vec3 getGyro() const { return _gyro; }
    Vec3 getRawAccel() const { return _raw_accel; }
    CalibrationStatus getCalibration()  const { return _calibration; }

    bool    isHealthy() const { return _healthy; }
    uint8_t getLastFault() const { return _last_fault; }

private:
    DFRobot_BNO055_IIC _sensor;
    Quaternion _quaternion;
    Orientation _orientation;
    Vec3 _linear_accel;
    Vec3 _gravity;    
    Vec3 _gyro;
    Vec3 _raw_accel;
    CalibrationStatus _calibration;
    
    bool    _healthy;
    uint8_t _last_fault;
    
    unsigned long _last_update_ms;
    static constexpr unsigned long UPDATE_INTERVAL_MS = 20;  // 50 Hz
};

#endif
