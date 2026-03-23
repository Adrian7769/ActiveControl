#include <Arduino.h>
#include "fram.h"

FRAM Fram; // Initialize FRAM
unsigned long lastRun = 0;
unsigned long timer = 10000; // 10 Seconds
void setup() {
    Serial.begin(115200); // Usb Serial Communication Rate
    Fram.begin();
    unsigned long PowerUpTime = millis();
}
int ADR = 0x00;
void loop() {
    if ((millis() - lastRun) >= timer) {
        lastRun = millis();
        Fram.WriteByte(0x0000,0xFF);
        Fram.ReadByte(0x0000);
    };
}
