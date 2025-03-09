/*
This file implements the Payload class to interface and control a 
9g micro servo. 

The inA pin controls the servo.
*/

#pragma once 

#include "Arduino.h"
#include "Servo.h"

class Payload {
    private:
        Servo servo;
        uint8_t servo_pin;
    
    public:
        Payload(uint8_t servo_pin);
        void begin(); 
        void ignite();
        void init_ignite();
        void init_flag();
        void raise_flag();
        void lower_flag();
};