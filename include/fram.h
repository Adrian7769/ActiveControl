#ifndef FRAM_H
#define FRAM_H
#include <Arduino.h>
#include <Wire.h>
class FRAM {
    public:
        FRAM();
        ~FRAM();
	void begin();

        // Getter and Setter Methods For Control Block Access
	uint8_t GetProgramState();
	bool SetProgramState(uint8_t byte);
	uint8_t GetErrorCodeByte();
	bool SetErrorCodeByte(uint8_t byte);
	uint8_t GetErrorCodeByte();
	uint16_t GetRecordCount();
	// Read and Write Blocks
	bool WriteDataBlock(DataBlock* block, uint8_t len); 

	// ---- NEEDS IMPLEMENTATION ---- //
        uint8_t ReadDataBlock(uint16_t adr);
	void DumpDataBytes();
	void ResetFram(); // Reset Fram
    private:
        uint32_t _clock_;
        uint16_t _address_;
        uint16_t _cursor_;
	// Private RawByteWrite and _WriteByte_
	uint8_t ReadControlBlockByte(uint16_t adr);
	bool WriteControlBlockByte(uint16_t adr, uint8_t byte);
        bool WriteByte(uint8_t byte);	
        uint8_t ReadByte(uint16_t adr);
        bool _WriteByte_(uint16_t adr, uint8_t byte); 
	bool RawByteWrite(uint16_t adr, uint8_t byte);
	// Probably will implement last
	void EnableWriteProtect(); // Pull pin high and disable writes this is used when memory is full.
	void DisableWriteProtect(); // Pull the pin low to keep writing enabled (This is the default)	
};
#endif
