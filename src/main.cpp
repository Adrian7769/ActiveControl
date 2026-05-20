#include <Arduino.h>
#include "fram.h"
#include <Wire.h>
#include "config.h"
FRAM Fram; // Initialize FRAM Instance
unsigned long lastRun = 0;
unsigned long timer = 10000; // 10 Seconds
void setup() {
    Serial.begin(115200); // Usb Serial Communication Rate
    unsigned long PowerUpTime = millis()			  
    delay(5000);
    // Fram and Datablock struct
    Fram.begin();
}

uint16_t ADR = 1;
void loop() {
// test loop write and read every 10 seconds
// For implementation we must be able to write the the chip
    if ((millis() - lastRun) >= timer) {
	lastRun = millis();
	//Serial.println(BUFFER_LENGTH);
	Fram.WriteBlock(&block);
    };
}
