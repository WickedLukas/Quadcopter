#include "common.h"

#include <core_pins.h>

// update LED
// mode: 0		off
// mode: 1		blink
// mode: > 1	on
void updateLED(uint8_t pin, uint8_t mode, uint32_t interval_ms) {
	static uint8_t ledState = LOW;
	static uint32_t t0_ms = 0, t_ms = 0;
	
	if (mode == 0) {
		ledState = LOW;
	}
	else if (mode == 1) {
		t_ms = millis();
		
		if ((t_ms - t0_ms) > interval_ms) {
		t0_ms = t_ms;
			
			if (ledState == LOW) {
				ledState = HIGH;
			}
			else {
				ledState = LOW;
			}
		}
	}
	else {
		ledState = HIGH;
	}
	
	// set LED state
	digitalWrite(pin, ledState);
}