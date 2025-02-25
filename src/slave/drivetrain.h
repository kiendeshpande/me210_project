/* 
This file implements the Drivetrain class which controls the four 
robot motors to achieve high-level chassis movements. Each robot 
motor is an instance of the Motor class (motor.h). 

The robot uses Mecanum wheels. Because of this, the chassis can move in 
the following ways: forward, backward, laterally, rotate, and stop. 

Motor 1 is the front left 
Motor 2 is the front right
Motor 3 is the back left
Motor 4 is the back right
*/

#pragma once 

#include "Arduino.h"
#include "motor.h"

class Drivetrain {
    private:
        uint8_t m1_inA; // m1 = motor1, etc.
        uint8_t m1_inB; 
        uint8_t m1_en;
        uint8_t m2_inA; 
        uint8_t m2_inB; 
        uint8_t m2_en; 
        uint8_t m3_inA; 
        uint8_t m3_inB; 
        uint8_t m3_en; 
        uint8_t m4_inA; 
        uint8_t m4_inB; 
        uint8_t m4_en;

        Motor m1; 
        Motor m2; 
        Motor m3; 
        Motor m4; 

    public: 
        Drivetrain(
            uint8_t m1_inA, // m1 = motor1, m2 = motor2, etc.
            uint8_t m1_inB, 
            uint8_t m1_en, 
            uint8_t m2_inA, 
            uint8_t m2_inB, 
            uint8_t m2_en, 
            uint8_t m3_inA, 
            uint8_t m3_inB, 
            uint8_t m3_en, 
            uint8_t m4_inA, 
            uint8_t m4_inB, 
            uint8_t m4_en
        );

        // 100 is currently passed in as the default duty cycle (~40%). 
        // TO DO: after assembly, determine the optimal speeds to move robot at. 
        void forward(uint8_t speed = 100); 
        void backward(uint8_t speed = 100); 
        void left(uint8_t speed = 100); 
        void right(uint8_t speed = 100); 
        void rot_cw(uint8_t speed = 100, uint8_t angle = 90);  // TO DO: fully implement this once chassis assembled 
        void rot_ccw(uint8_t speed = 100, uint8_t angle = 90); // TO DO: fully implement this once chassis assembled
        void stop(); 
};