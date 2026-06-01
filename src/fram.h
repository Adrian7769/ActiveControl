#ifndef FRAM_H
#define FRAM_H
#include <Arduino.h>
#include <Wire.h>

struct __attribute__((packed)) DataBlock {
    // ---- Time (4 bytes) ----
    uint32_t timestamp_ms;          // millis()
    // ---- Barometer (4 bytes) ----
    uint32_t pressure_pa;           // raw pressure
    // ---- Quaternion (8 bytes) ----
    int16_t  qw;                    // scaled: real_value * 16384
    int16_t  qx;                    // (BNO055 native scaling, no precision loss)
    int16_t  qy;
    int16_t  qz;
    // ---- Linear acceleration (6 bytes) ----
    int16_t  accel_x;               // milli-g (mg)
    int16_t  accel_y;
    int16_t  accel_z;
    // ---- Guidance outputs (3 bytes) ----
    int8_t   pid_pitch;             // -100 to +100 (normalized * 100)
    int8_t   pid_yaw;
    int8_t   pid_roll;
    // ---- Fin positions (4 bytes) ----
    uint8_t  fin_0;                 // degrees 0-180
    uint8_t  fin_1;
    uint8_t  fin_2;
    uint8_t  fin_3;
    // ---- System (2 bytes) ----
    uint8_t  flight_state;
    uint8_t  flags;
    //        bit 0: guidance enabled
    //        bit 1: FRAM >90% full
    //        bit 2: IMU unhealthy
    //        bit 3: baro unhealthy
    //        bit 4: servo unhealthy
    //        bits 5-7: reserved
    uint8_t  _reserved;             
};

enum class ProgramState : uint8_t {
    UNINITIALIZED = 0x00,
    IDLE          = 0x01,
    FAULT         = 0x02,
    ARMED         = 0x03,
    ASCENT        = 0x04,
    APOGEE        = 0x05,
    DESCENT       = 0x06,
    LANDED        = 0x07,
    POST_FLIGHT   = 0x08,
};

class FRAM {
public:
    FRAM();
    ~FRAM();
    void begin();

    // Control block access
    uint8_t  GetProgramState();
    bool     SetProgramState(uint8_t byte);
    uint8_t  GetErrorCodeByte();
    bool     SetErrorCodeByte(uint8_t byte);
    uint16_t GetRecordCount();
    bool     IsFull();

    bool WriteDataBlock(const DataBlock* block); 
    bool ReadDataBlock(uint16_t index, DataBlock* out);

    void DumpDataBytes();
    void DumpControlBlock();
    void ResetFram();
    // testing functions
    void test();
    void controlBlockSanityTest();
    void overFlowTest();

private:
    uint32_t _clock_;
    uint16_t _address_;
    uint16_t _cursor_;  

    bool     WriteBytes(uint16_t addr, const uint8_t* data, size_t len);
    bool     ReadBytes (uint16_t addr, uint8_t* out,        size_t len);
    uint8_t  ReadControlBlockByte(uint16_t addr);
    bool     WriteControlBlockByte(uint16_t addr, uint8_t byte);

    // Helpers
    uint16_t BlockIndexToAddr(uint16_t index);
    bool     PersistCursor();
    bool     PersistRecordCount(uint16_t count);
};
#endif
