#ifndef FRAM_H
#define FRAM_H
#include <Arduino.h>
#include <Wire.h>
class FRAM {
    public:
        FRAM();
        ~FRAM();
        byte ReadByte(uint16_t adr);
        void begin();
        bool WriteByte(uint16_t adr, byte data);
    private:
        uint32_t _clock_;
        uint8_t _address_;
        uint16_t _capacity_;
        uint16_t _cursor_;
};
#endif