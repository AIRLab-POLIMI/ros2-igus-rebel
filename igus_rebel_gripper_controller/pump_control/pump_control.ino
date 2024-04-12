// Blinking the internal LED on pin 13
// give permissions for flashing and monitoring serial communication
//  sudo chmod a+rw /dev/ttyACM0

#include <Arduino.h>

void grip();
void release();
void pump_off();

void setup() {

	// Set digital pins as outputs for controlling relays
	pinMode(4, OUTPUT); // relay 1 --> pin 1 valve 2
	pinMode(5, OUTPUT); // relay 2 --> pin 3 VDD
	pinMode(6, OUTPUT); // relay 3 --> pin 5 valve 1

	Serial.begin(115200);

	pump_off();
}

void loop() {
	// wait for user input via serial
	// if serial avaialble, read it and execute the corresponding function
	if (Serial.available() > 0) {
		String command = Serial.readStringUntil('\n');
		if (command == "grip") {
			grip();
		} else if (command == "release") {
			release();
		} else if (command == "off") {
			pump_off();
		}
	} else {
		delay(10); // wait for some user input
	}
}

// grip function
void grip() {
	digitalWrite(4, HIGH); // or HIGH
	digitalWrite(5, LOW);
	digitalWrite(6, HIGH);
}

// release function
void release() {
	digitalWrite(4, LOW);
	digitalWrite(5, LOW);
	digitalWrite(6, LOW);
}

// pump off function
void pump_off() {
	digitalWrite(4, HIGH);
	digitalWrite(5, HIGH);
	digitalWrite(6, HIGH);
}
