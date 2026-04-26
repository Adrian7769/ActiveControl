#include <Arduino.h>
#include "fram.h"
#include <Wire.h>
#include "config.h"
FRAM Fram; // Initialize FRAM
unsigned long lastRun = 0;
unsigned long timer = 10000; // 10 Seconds
void setup() {
    Serial.begin(115200); // Usb Serial Communication Rate
    Fram.init();
    unsigned long PowerUpTime = millis();
}
uint16_t ADR = 0;
void loop() {
// test loop write and read every 10 seconds
// For implementation we must be able to write the the chip 
    if ((millis() - lastRun) >= timer) {
	lastRun = millis();
	if(Fram.WriteByte(ADR,0xFF)) {
		// test read functionality
		Fram.ReadByte(ADR);
	} else {
		Serial.print("FRAM_WRITE_FAILURE");
	}
    };
}
