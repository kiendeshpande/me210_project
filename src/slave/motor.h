/*
This file implements the Motor class to interface and control a 
DC brushed motor via the L298 H-bridge module. 

The inA and inB pins control polarity (direction) and the enable 
pin is a PWM signal to achieve speed control. 
*/

#pragma once 

#include "Arduino.h"

class Motor {
    private: 
        uint8_t inA;
        uint8_t inB; 
        uint8_t enable;

    public: 
        Motor(uint8_t inA, uint8_t inB, uint8_t enable); 
        void forward(uint8_t speed); 
        void backward(uint8_t speed); 
        void stop(); 
};

