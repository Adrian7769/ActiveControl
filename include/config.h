// MB85RC256V
#define FRAM_ADR 0x50
#define FRAM_CLOCK 50000 // 50 kHz
#define FRAM_MAX_RECORD 1023 // Max Records
#define FRAM_MAGIC_VAL 0x45 // 69
// Control Block Structure (16 bytes)
// [0] Cursor MSB
// [1] Cursor LSB
// [2] Program State Byte
// [3] Write Protect Byte
// [4] Error Code Byte
// [5] Memory Full byte 
// [6] Record Count MSB
// [7] Record Count LSB
// [8] Magic Byte
// [9...15] Reserved
#define FRAM_CONTROLBLOCK_START 0x0000 // Control block start
#define FRAM_CURSOR_MSB 0x0000 // Cursor LSB
#define FRAM_CURSOR_LSB 0x0001 // Cursor MSB
#define FRAM_STATE_BYTE 0x0002 // Curent State
#define FRAM_WP_BYTE 0x0003 // is Write Protect Enabled
#define FRAM_ERROR_BYTE 0x0004 // Log Latest Error
#define FRAM_FULL_BYTE 0x0005 // Are we full?
#define FRAM_RECORD_COUNT_MSB 0x0006 // Record Count MSB
#define FRAM_RECORD_COUNT_LSB 0x0007 // Record Count LSB
#define FRAM_MAGIC_BYTE 0x0008 // Magic Byte
#define FRAM_CONTROLBLOCK_END 0x000F // 15th Byte
// Data Block Structure (32 Bytes)
struct DataBlock {
	uint16_t PUtime;
	uint16_t AltitudeByte;
	uint8_t AccelY;
	uint8_t AccelX;
};

#define FRAM_DATABLOCK_START 0x0010 // 16th byte
#define FRAM_DATABLOCK_END 0x7FFF // (32,768 bytes)
// Start writing data upon liftoff -> landing
// ESP32 PIN MAP
#define ESP_SDA_PIN 21
#define ESP_SCL_PIN 22
#define FRAM_WP_PIN 23

// Debug
#define DIAG_FRAM
// Data Structures
