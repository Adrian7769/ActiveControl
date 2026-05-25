#ifndef BAROMETER_H
#define BAROMETER_H

#include <DFRobot_BMP280.h>

class Barometer {
public:
    Barometer();
    uint8_t begin();
    void tick();
    float getPressurePa() const { 
	    return _pressure_pa; 
    }
    float getTemperatureC() const { 
	    return _temperature_c; 
    }
    float getAltitudeM() const { 
	    return _altitude_m; 
    }
    void  setSeaLevelPressure(float pa) {
	    _sea_level_pa = pa; 
    }
    float getSeaLevelPressure() const   { 
	    return _sea_level_pa; 
    }
    bool    isHealthy()    const { 
	    return _healthy; 
    }
    uint8_t getLastFault() const { 
	    return _last_fault; 
    }
private:
    DFRobot_BMP280_IIC _sensor;
    float _pressure_pa;
    float _temperature_c;
    float _altitude_m;
    float _sea_level_pa;
    bool    _healthy;
    uint8_t _last_fault;
    unsigned long _last_update_ms;
    static constexpr unsigned long UPDATE_INTERVAL_MS = 50;  // 20 Hz
};
#endif

