
// Major.Minor.Patch
constexpr char VERSION[] = "1.0.0";

// MB85RC256V
constexpr uint8_t FRAM_ADR = 0x50; //constexpr must be computed at compile time
constexpr uint32_t FRAM_CLOCK = 50000; // 50 kHz
constexpr uint16_t FRAM_MAX_RECORD = 1023; // Max Records (32 byte data blocks)
constexpr uint8_t FRAM_MAGIC_VAL = 0x45; // 69
constexpr uint8_t FRAM_DATABLOCK_SIZE = 32; // size of 32 bytes
constexpr uint8_t FRAM_CONTROLBLOCK_SIZE = 16;
constexpr uint16_t FRAM_CONTROLBLOCK_START = 0x0000; // Control block start
						    
constexpr uint16_t FRAM_CURSOR_MSB = 0x0000; // Cursor LSB
constexpr uint16_t FRAM_CURSOR_LSB = 0x0001; // Cursor MSB
constexpr uint16_t FRAM_PROGRAMSTATE_BYTE = 0x0002; // Curent State
constexpr uint16_t FRAM_ERROR_BYTE = 0x0003; // Log Latest Error
constexpr uint16_t FRAM_FULL_BYTE = 0x0004; // Are we full?
constexpr uint16_t FRAM_RECORD_COUNT_MSB = 0x0005; // Record Count MSB
constexpr uint16_t FRAM_RECORD_COUNT_LSB = 0x0006; // Record Count LSB
constexpr uint16_t FRAM_MAGIC_BYTE = 0x0007;  // Magic Byte
constexpr uint16_t FRAM_CONTROLBLOCK_END = 0x000F; // 15th Byte
// Data Block Structure (32 Bytes)
constexpr uint16_t FRAM_DATABLOCK_START = 0x0010; // 16th byte
constexpr uint16_t FRAM_DATABLOCK_END = 0x7FFF; // (32,768 bytes)
// Start writing data upon liftoff -> landing
// ESP32 PIN MAP
constexpr uint8_t ESP_SDA_PIN = 21;
constexpr uint8_t ESP_SCL_PIN = 22;

// Debug
#define DIAG_FRAM

