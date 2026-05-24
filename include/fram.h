#ifndef FRAM_H
#define FRAM_H
#include <Arduino.h>
#include <Wire.h>

struct __attribute__((packed)) DataBlock {
    uint32_t timestamp_ms;
    uint32_t pressure_pa;
    float    accel_x;
    float    accel_y;
    float    accel_z;
    int16_t  euler_head;
    int16_t  euler_roll;
    int16_t  euler_pitch;
    int16_t  temp_c_x100;
    uint8_t  flight_state;
    uint8_t  flags;
    uint8_t  crc;
    uint8_t  reserved;
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
