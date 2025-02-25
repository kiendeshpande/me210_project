#pragma once 

#include "Arduino.h"

class Ultrasonic {
	private:
		uint8_t trigger_pin; 
		uint8_t echo_pin;
		uint16_t max_distance = 400;
		uint16_t speed_sound  = 340;  


	public:
		Ultrasonic(uint8_t trigger_pin, uint8_t echo_pin); 
		distance(); 
}