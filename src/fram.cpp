#include "fram.h"
#include <wire.h>
#include "config.h"
FRAM::FRAM() { // Default 1MHz
    _clock_ = FRAM_CLOCK;
    _address_ = FRAM_ADR;
};
FRAM::~FRAM() {
    Wire.end();
};
void FRAM::begin() {
    Wire.begin();
    Wire.setClock(_clock_);
};
byte FRAM::ReadByte(uint16_t adr) {
    Wire.beginTransmission(_address_);
    Wire.write((adr >> 8) & 0xFF); // High Byte
    Wire.write(adr & 0xFF); // Low Byte
    Wire.endTransmission(false);
    Wire.requestFrom(_address_,1U);
    if (Wire.available()) {
        byte data = Wire.read();
        Serial.print("Read 0x");
        Serial.print(data,HEX);
        Serial.println("From Address 0x");
        Serial.print(adr);
        return data;
    } else {
        Serial.println("No Available Data, Returning 0");
        return 0;
    }
};

bool FRAM::WriteByte(uint16_t adr, byte data) {
    Wire.beginTransmission(_address_);
    Wire.write((adr >> 8) & 0xFF); // High Byte of memory address
    Wire.write(adr & 0xFF); // Write Low Byte of memory address
    Wire.write(data);
    int result = Wire.endTransmission();
    // Returns 0 for success
    return (result == 0);
};
