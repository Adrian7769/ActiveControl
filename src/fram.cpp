#include "fram.h"
#include <Wire.h>
#include "config.h"

// ---------- CONSTRUCTOR -------- //
FRAM::FRAM() { 
    _clock_ = FRAM_CLOCK;
    _address_ = FRAM_ADR;
    _cursor_ = FRAM_DATABLOCK_START; // Set to Datablock 0 by Default
};
// --------- DESTRUCTOR -------- //
FRAM::~FRAM() {
    Wire.end();
};
// -------- PUBLIC METHODS -------- //
void FRAM::begin() {
    Wire.begin(ESP_SDA_PIN,ESP_SCL_PIN);
    Wire.setClock(_clock_);
    // If Control Block Not Initialized With Magic Byte
    if (FRAM_MAGIC_VAL != ReadControlBlockByte(FRAM_MAGIC_BYTE)) {
	// Update Magic Byte To Magic Value 
	if (WriteControlBlockByte(FRAM_MAGIC_BYTE, FRAM_MAGIC_VAL)) {
	    #ifdef DIAG_FRAM
	    Serial.print("Updated Magic Byte to: 0x");Serial.print(FRAM_MAGIC_VAL,HEX);Serial.println("");
	    #endif
	} else {
	    #ifdef DIAG_FRAM 
	    Serial.print("Failed to Update Magic Byte!");
	    #endif
	
	// Update Cursor to The start of the Datablock 
	if(WriteControlBlockByte(FRAM_CURSOR_MSB, (_cursor_ >> 8) & 0xFF) && WriteControlBlockByte(FRAM_CURSOR_LSB, (_cursor_ & 0xFF)) ) {
	    #ifdef DIAG_FRAM
	    Serial.print("Updated Cursor to: 0x");Serial.print(FRAM_DATABLOCK_START,HEX);Serial.println("");
	    #endif
	} else {
	    #ifdef DIAG_FRAM 
	    Serial.print("Failed to Update Cursor!");
	    #endif             	
        };
    } else { 
	    // Update the Cursor to next available Block
	    _cursor_ = (ReadControlBlockByte(FRAM_CURSOR_MSB) << 8) | ReadControlBlockByte(FRAM_CURSOR_LSB);
	}
};
uint8_t FRAM::GetProgramState() {
	return ReadControlBlockByte(FRAM_STATE_BYTE);
};
bool FRAM::SetProgramState(uint8_t byte) {
	return (WriteControlBlockByte(FRAM_STATE_BYTE, byte) == 0);
};
uint8_t FRAM::GetErrorCodeByte() {
	return ReadControlBlockByte(FRAM_ERROR_BYTE);
};
bool FRAM::SetErrorCodeByte(uint8_t byte) {
	return (WriteControlBlockByte(FRAM_ERROR_BYTE,byte) == 0);
};
uint16_t FRAM::GetRecordCount() {
	return ((ReadControlBlockByte(FRAM_RECORD_COUNT_MSB) << 8) | 
			ReadControlBlockByte(FRAM_RECORD_COUNT_LSB));
};
bool WriteDataBlock(DataBlock* block, uint8_t len) {
	for (int i = 0; i < std::sizeof(block); i++) {
		WriteByte(block[i]);
	}

}
// -------- PRIVATE METHODS -------- //
uint8_t FRAM::ReadControlBlockByte(uint16_t adr) {
	if (adr >= FRAM_CONTROLBLOCK_START && adr <= (FRAM_CONTROLBLOCK_END)) {
		return ReadByte(adr);
	} else {
		#ifdef DIAG_FRAM
		Serial.println("Out Of Control Block Bounds READ. Returning 0x0000");
		return 0x0000;
		#endif
	}
};
bool FRAM::WriteControlBlockByte(uint16_t adr, uint8_t byte) {
	if (adr >= FRAM_CONTROLBLOCK_START && adr <= (FRAM_CONTROLBLOCK_END)) {
		return _WriteByte_(adr,byte) == 0; // successful write to control block
	} else {
		#ifdef DIAG_FRAM
		Serial.println("Out Of Control Block Bounds WRITE. Returning False");
		#endif
		return false;
	}
};

uint8_t FRAM::ReadByte(uint16_t adr) {
    Wire.beginTransmission(_address_);
    // Que Start
    Wire.write((adr >> 8) & 0xFF); // High Byte
    Wire.write(adr & 0xFF); // Low Byte
    // Que End
    Wire.endTransmission(false); // Send Request Keep Bus Active (Send Restart)
    Wire.requestFrom(_address_,1U); // Request 1 Byte
    if (Wire.available()) { // Read Byte from buffer
        uint8_t data = Wire.read();
        #ifdef DIAG_FRAM
	Serial.print("Read: 0x");
	Serial.print(data,HEX);
	Serial.print(" From Address: 0x");
	Serial.print(adr);Serial.println("");
	#endif
        return data;
    } else {
	#ifdef DIAG_FRAM
	Serial.println("No Available Data, Returning 0");
	#endif
        return 0x0000;
    }
};
bool FRAM::_WriteByte_(uint16_t adr, uint8_t byte) {
    Wire.beginTransmission(_address_);
    // Queue Start
    Wire.write((adr >> 8) & 0xFF); // High Byte of memory address
    Wire.write(adr & 0xFF); // Write Low Byte of memory address
    Wire.write(byte); // data
    // Queue End
    int result = Wire.endTransmission(); // SEND TO MB85RC256V
    if (adr >= FRAM_DATABLOCK_START && adr <= FRAM_DATABLOCK_END) {
	    _cursor_ = adr + 1; //increment the cursor to the next Data Block
	    if(RawByteWrite(FRAM_CURSOR_MSB, (_cursor_ >> 8) & 0xFF) && RawByteWrite(FRAM_CURSOR_LSB, (_cursor_ & 0xFF))) {
		#ifdef DIAG_FRAM
		Serial.print("Updated Cursor to: 0x");Serial.print(_cursor_,HEX);Serial.println("");
		#endif
	    } else {
		#ifdef DIAG_FRAM 
	        Serial.print("Failed to Update Cursor!");
		#endif
	    }
    } else { //Write to Control Block
	    #ifdef DIAG_FRAM
	    Serial.print("Result: ");Serial.print(result);Serial.print(" Wrote: 0x");Serial.print(byte,HEX);Serial.print(" To ADR: 0x" ); Serial.print(adr); Serial.println("");
	    #endif
    };
    return (result == 0);
};
bool FRAM::WriteByte(uint8_t byte) {
	return _WriteByte_(_cursor_,byte);
}
bool FRAM::RawByteWrite(uint16_t adr, uint8_t byte) {
    Wire.beginTransmission(_address_);
    // Queue Start
    Wire.write((adr >> 8) & 0xFF); // High Byte of memory address
    Wire.write(adr & 0xFF); // Write Low Byte of memory address
    Wire.write(byte); // data
    // Queue End
    return (Wire.endTransmission() == 0); // SEND TO MB85RC256V
};
