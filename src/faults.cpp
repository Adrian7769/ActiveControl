#include "faults.h"

const FaultEntry FAULT_TABLE[] = {
    { FAULT_NONE, "NONE", "No fault" },
    { FAULT_FRAM_WRITE, "FRAM_WRITE", "FRAM write failed" },
    { FAULT_FRAM_READ, "FRAM_READ", "FRAM read failed" },
    { FAULT_FRAM_FULL, "FRAM_FULL", "FRAM memory full" },
    { FAULT_BMP_NOT_FOUND_LOCAL, "BMP_NF", "BMP280 not found"},
    { FAULT_BMP_TIMEOUT_LOCAL, "BMP_T", "BPM280 timeout"},
    { FAULT_BNO_NOT_FOUND_LOCAL, "BNO_NF", "BNO055 not found"},
    { FAULT_BNO_TIMEOUT_LOCAL, "BNO_T", "BNO055 timeout"},
    { FAULT_SERVO_INIT_LOCAL, "SERVO_INIT", "Servo init failed" }
};
const size_t FAULT_TABLE_SIZE = sizeof(FAULT_TABLE) / sizeof(FAULT_TABLE[0]);

const FaultEntry* lookupFault(uint8_t code) {
    for (size_t i = 0; i < FAULT_TABLE_SIZE; ++i) {
        if (FAULT_TABLE[i].code == code) {
            return &FAULT_TABLE[i];
        }
    }
    return nullptr;
}
