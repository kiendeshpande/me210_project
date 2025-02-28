/*
Implementation of class for HC-SR04 ultrasonic sensor. 
Supports triggering a pulse and calculating distance, in cm, from the echo.
*/


#pragma once 

#include "Arduino.h"

class Ultrasonic {
	private:
		uint8_t trigger_pin; 
		uint8_t echo_pin;
		uint16_t max_distance = 400; // HC-SR04 spec 

	public:
		Ultrasonic(uint8_t trigger_pin, uint8_t echo_pin); 
    void begin();
		float distance(); 
};