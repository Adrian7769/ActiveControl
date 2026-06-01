#include "fram.h"
#include "config.h"
#include "faults.h"

FRAM::FRAM() {
    _clock_   = FRAM_CLOCK;
    _address_ = FRAM_ADR;
    _cursor_  = 0;  
}

FRAM::~FRAM() { 
	Wire.end(); 
}

void FRAM::begin() {
    delay(100);
    Wire.setClock(_clock_);
    uint8_t magic = 0;
    if (!ReadBytes(FRAM_MAGIC_BYTE, &magic, 1)) {
#ifdef DIAG_FRAM
		    Serial.println("FRAM bus not responding, skipping init");
#endif
		    return;
    }
    if (magic != FRAM_MAGIC_VAL) {
	// Fresh boot initialize the control block
        WriteControlBlockByte(FRAM_MAGIC_BYTE, FRAM_MAGIC_VAL);
        WriteControlBlockByte(FRAM_CURSOR_MSB, 0);
        WriteControlBlockByte(FRAM_CURSOR_LSB, 0);
        WriteControlBlockByte(FRAM_RECORD_COUNT_MSB, 0);
        WriteControlBlockByte(FRAM_RECORD_COUNT_LSB, 0);
        WriteControlBlockByte(FRAM_FULL_BYTE, 0);
        WriteControlBlockByte(FRAM_ERROR_BYTE, 0);
	WriteControlBlockByte(FRAM_PROGRAMSTATE_BYTE, 0); // uninit
     _cursor_ = 0;
#ifdef DIAG_FRAM
        Serial.println("FRAM initialized fresh");
#endif
    } else {
        _cursor_ = (ReadControlBlockByte(FRAM_CURSOR_MSB) << 8) |  ReadControlBlockByte(FRAM_CURSOR_LSB);
#ifdef DIAG_FRAM
        Serial.print("FRAM resumed at block index ");
        Serial.println(_cursor_);
#endif
    }
}

bool FRAM::WriteDataBlock(const DataBlock* block) {
    if (_cursor_ >= FRAM_MAX_RECORD) {
        SetErrorCodeByte(FAULT_FRAM_FULL);
        WriteControlBlockByte(FRAM_FULL_BYTE, 1); 
	WriteControlBlockByte(FRAM_PROGRAMSTATE_BYTE,static_cast<uint8_t>(ProgramState::FAULT));
        return false;
    }
    uint16_t addr = BlockIndexToAddr(_cursor_);
    if (!WriteBytes(addr, reinterpret_cast<const uint8_t*>(block), FRAM_DATABLOCK_SIZE)) {
        SetErrorCodeByte(FAULT_FRAM_WRITE);
        return false;
    }
    _cursor_++;
    if (!PersistCursor()) {
	    return false;
    }
    if (!PersistRecordCount(_cursor_)) {
	    return false;
    }
    return true;
}
void FRAM::DumpDataBytes() {
	uint16_t rec = GetRecordCount();
	uint8_t buff[FRAM_DATABLOCK_SIZE];
	// Dump ALLDataBlocks
	for (uint16_t i = 0; i < rec; ++i) {
		if (!ReadBytes(BlockIndexToAddr(i), &buff[0], FRAM_DATABLOCK_SIZE)) {
#ifdef DIAG_FRAM
			Serial.print("Read Failed at Record ");
			Serial.println(i);
#endif
			continue;
		}
		
		//Serial.print(i);
		//Serial.print(": ");
		for (size_t b = 0; b < FRAM_DATABLOCK_SIZE; ++b) {
		    if (buff[b] < 0x10) Serial.print('0');  // leading zero
		    Serial.print(buff[b], HEX);
		    Serial.print(' ');
		}
		Serial.println();
    	}
}

void FRAM::DumpCSV() {
    uint16_t rec = GetRecordCount();
    DataBlock block;

    Serial.println(F("time_ms,pressure_pa,qw,qx,qy,qz,accel_x_mg,accel_y_mg,accel_z_mg,pid_pitch,pid_yaw,pid_roll,fin0,fin1,fin2,fin3,state,flags"));

    for (uint16_t i = 0; i < rec; ++i) {
        if (!ReadDataBlock(i, &block)) {
#ifdef DIAG_FRAM
            Serial.print(F("# READ FAIL record ")); Serial.println(i);
#endif
            continue;
        }

 
        Serial.print(block.timestamp_ms); Serial.print(',');
        Serial.print(block.pressure_pa);  Serial.print(',');

        Serial.print(block.qw / 16384.0f, 4); Serial.print(',');
        Serial.print(block.qx / 16384.0f, 4); Serial.print(',');
        Serial.print(block.qy / 16384.0f, 4); Serial.print(',');
        Serial.print(block.qz / 16384.0f, 4); Serial.print(',');

        Serial.print(block.accel_x); Serial.print(',');
        Serial.print(block.accel_y); Serial.print(',');
        Serial.print(block.accel_z); Serial.print(',');

        Serial.print(block.pid_pitch / 100.0f, 2); Serial.print(',');
        Serial.print(block.pid_yaw   / 100.0f, 2); Serial.print(',');
        Serial.print(block.pid_roll  / 100.0f, 2); Serial.print(',');

        Serial.print(block.fin_0); Serial.print(',');
        Serial.print(block.fin_1); Serial.print(',');
        Serial.print(block.fin_2); Serial.print(',');
        Serial.print(block.fin_3); Serial.print(',');

        Serial.print(block.flight_state); Serial.print(',');
        Serial.println(block.flags);
    }
}
void FRAM::DumpControlBlock() {
    uint8_t buff[FRAM_CONTROLBLOCK_SIZE];

    if (!ReadBytes(FRAM_CONTROLBLOCK_START, buff, FRAM_CONTROLBLOCK_SIZE)) {
        Serial.println(F("Read failed for control block"));
        return;
    }

    for (size_t b = 0; b < FRAM_CONTROLBLOCK_SIZE; ++b) {
        if (buff[b] < 0x10) Serial.print('0');
        Serial.print(buff[b], HEX);
        Serial.print(' ');
    }
    Serial.println();
}
bool FRAM::ReadDataBlock(uint16_t index, DataBlock* out) {
    if (index >= FRAM_MAX_RECORD || out == nullptr) {
	    return false;
    }
    return ReadBytes(BlockIndexToAddr(index), reinterpret_cast<uint8_t*>(out), FRAM_DATABLOCK_SIZE);
}
bool FRAM::WriteBytes(uint16_t addr, const uint8_t* data, size_t len) {
    Wire.beginTransmission(_address_);
    Wire.write((addr >> 8) & 0xFF);
    Wire.write( addr       & 0xFF);
    for (size_t i = 0; i < len; ++i) {
	    Wire.write(data[i]);
    }
    return Wire.endTransmission() == 0;
}
bool FRAM::ReadBytes(uint16_t addr, uint8_t* out, size_t len) {
    Wire.beginTransmission(_address_);
    Wire.write((addr >> 8) & 0xFF);
    Wire.write( addr       & 0xFF);
    if (Wire.endTransmission(false) != 0) {
	    return false;
    }
    size_t got = Wire.requestFrom(_address_, (uint8_t)len);
    if (got != len) {
	    return false;
    }
    for (size_t i = 0; i < len; ++i){
	    out[i] = Wire.read();
    }
    return true;
}
bool FRAM::PersistCursor() {
    return WriteControlBlockByte(FRAM_CURSOR_MSB, (_cursor_ >> 8) & 0xFF) && WriteControlBlockByte(FRAM_CURSOR_LSB,  _cursor_ & 0xFF);
}
bool FRAM::PersistRecordCount(uint16_t count) {
    return WriteControlBlockByte(FRAM_RECORD_COUNT_MSB, (count >> 8) & 0xFF) && WriteControlBlockByte(FRAM_RECORD_COUNT_LSB, count & 0xFF);
}
uint8_t FRAM::ReadControlBlockByte(uint16_t addr) {
    if (addr > FRAM_CONTROLBLOCK_END) {
	    return 0; // This might come back to bite me :*(
    };
    uint8_t b = 0;
    ReadBytes(addr, &b, 1);
    return b;
}
bool FRAM::WriteControlBlockByte(uint16_t addr, uint8_t byte) {
    if (addr > FRAM_CONTROLBLOCK_END) {
	    return false;
    };
    return WriteBytes(addr, &byte, 1);
}
void FRAM::ResetFram() {
    WriteControlBlockByte(FRAM_CURSOR_MSB, 0);
    WriteControlBlockByte(FRAM_CURSOR_LSB, 0);
    WriteControlBlockByte(FRAM_RECORD_COUNT_MSB, 0);
    WriteControlBlockByte(FRAM_RECORD_COUNT_LSB, 0);
    WriteControlBlockByte(FRAM_FULL_BYTE, 0);
    WriteControlBlockByte(FRAM_ERROR_BYTE, 0);
    WriteControlBlockByte(FRAM_PROGRAMSTATE_BYTE, static_cast<uint8_t>(ProgramState::IDLE));
    _cursor_ = 0;

#ifdef DIAG_FRAM
    Serial.println(F("FRAM reset complete"));
#endif
}
// ---- TESTING / SANITY METHODS ---- //
void FRAM::controlBlockSanityTest() {
	}
void FRAM::overFlowTest() {
}
// ---- PUBLIC CONTROL BLOCK ACCESS ---- //
uint8_t FRAM::GetProgramState() { 
	return ReadControlBlockByte(FRAM_PROGRAMSTATE_BYTE); 
}
bool FRAM::SetProgramState(uint8_t b) { 
	return WriteControlBlockByte(FRAM_PROGRAMSTATE_BYTE, b); 
}
uint8_t FRAM::GetErrorCodeByte() { 
	return ReadControlBlockByte(FRAM_ERROR_BYTE); 
}
bool FRAM::SetErrorCodeByte(uint8_t b) { 
	return WriteControlBlockByte(FRAM_ERROR_BYTE, b); 
}
bool FRAM::IsFull() { 
	return ReadControlBlockByte(FRAM_FULL_BYTE) != 0; 
}
uint16_t FRAM::GetRecordCount() {
    return (ReadControlBlockByte(FRAM_RECORD_COUNT_MSB) << 8) |  ReadControlBlockByte(FRAM_RECORD_COUNT_LSB);
}
// ---- HELPER ---- //
uint16_t FRAM::BlockIndexToAddr(uint16_t index) {
	return FRAM_DATABLOCK_START + (index * FRAM_DATABLOCK_SIZE);
}
