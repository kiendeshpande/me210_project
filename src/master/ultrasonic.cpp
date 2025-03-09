/*
Implementation of class for HC-SR04 ultrasonic sensor. 
Supports triggering a pulse and calculating distance, in cm, from the echo.
*/

#include "ultrasonic.h"
#include "Arduino.h"


Ultrasonic::Ultrasonic(uint8_t trigger_pin, uint8_t echo_pin) 
    : trigger_pin(trigger_pin), echo_pin(echo_pin) {}


/* Called during setup to initialize ultrasonic pins */
void Ultrasonic::begin() {
    pinMode(trigger_pin, OUTPUT); 
    pinMode(echo_pin, INPUT);   
    digitalWrite(trigger_pin, LOW); // init trigger pin to LOW
}

/* Returns distance in centimeters from sensor to detected object. */
float Ultrasonic::distance() {
	digitalWrite(trigger_pin, HIGH); 
	delayMicroseconds(10);  // pin must be high for at least 10us to send pulse
	digitalWrite(trigger_pin, LOW); 

	unsigned long duration = pulseIn(echo_pin, HIGH); 

  float distance = duration / 58.0; 

	if (distance > max_distance) 
		return -1.0;

	return distance; 
}