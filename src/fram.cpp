#include "fram.h"
#include <Wire.h>
#include "config.h"

// Control Block First N Bytes of memory Reserved
// Open Questions:
// Resolution of Data -> How Often we Write To memory
// How often can we write to memory? We cannot exceed 1MHz
// Lets assume this is not an issue for now
// First Bytes 0 - 16 reserved for control block (cursor Where was i last?)
// The structure of this control block may be as follows:
// MEMORY CONTROL BLOCK STRUCTURE BYTES 0 - 16
// [0] Last Write Address;
// [1] Last Read Address;
// [2] Is Full Bool;

FRAM::FRAM() { 
    _clock_ = FRAM_CLOCK;
    _address_ = FRAM_ADR;
};
FRAM::~FRAM() {
    Wire.end();
};
void FRAM::init() {
    Wire.begin(ESP_SDA,ESP_SCL);
    Wire.setClock(_clock_);
    // Control block and cursor
};

// dump FRAM METHOD Implementation
// Memory Overflow Guard

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
    // Queue Start
    Wire.write((adr >> 8) & 0xFF); // High Byte of memory address
    Wire.write(adr & 0xFF); // Write Low Byte of memory address
    Wire.write(data); // data
    // Queue End
    int result = Wire.endTransmission(); // SEND TO MB85RC256V
#ifdef DIAG_FRAM
	    Serial.print("Result: ");Serial.print(result);Serial.print(" Wrote: ");Serial.print(data);Serial.print(" To ADR: " ); Serial.print(adr); Serial.println("");
#endif
    return (result == 0);
};
