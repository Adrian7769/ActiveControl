#ifndef FAULTS_H
#define FAULTS_H

#include <cstdint>
#include <cstddef>

constexpr uint8_t FAULT_NONE                  = 0x00;
constexpr uint8_t FAULT_FRAM_WRITE            = 0x01;
constexpr uint8_t FAULT_FRAM_READ             = 0x02;
constexpr uint8_t FAULT_FRAM_FULL             = 0x03;

constexpr uint8_t FAULT_BMP_NOT_FOUND_LOCAL = 0x20;
constexpr uint8_t FAULT_BMP_TIMEOUT_LOCAL   = 0x21;

constexpr uint8_t FAULT_BNO_NOT_FOUND_LOCAL = 0x10;
constexpr uint8_t FAULT_BNO_TIMEOUT_LOCAL   = 0x11;

constexpr uint8_t FAULT_PCA_NOT_FOUND_LOCAL = 0x30;
constexpr uint8_t FAULT_PCA_TIMEOUT_LOCAL   = 0x31;

struct FaultEntry {
    uint8_t     code;
    const char* name;
    const char* description;
};
extern const FaultEntry FAULT_TABLE[];
extern const size_t     FAULT_TABLE_SIZE;
const FaultEntry* lookupFault(uint8_t code);

#endif
