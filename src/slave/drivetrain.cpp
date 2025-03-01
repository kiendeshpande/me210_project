#include "drivetrain.h"

Drivetrain::Drivetrain(uint8_t m1_inA, uint8_t m1_inB, uint8_t m1_en, 
                       uint8_t m2_inA, uint8_t m2_inB, uint8_t m2_en, 
                       uint8_t m3_inA, uint8_t m3_inB, uint8_t m3_en, 
                       uint8_t m4_inA, uint8_t m4_inB, uint8_t m4_en)
    : m1_inA(m1_inA), m1_inB(m1_inB), m1_en(m1_en), 
    m2_inA(m2_inA), m2_inB(m2_inB), m2_en(m2_en), 
    m3_inA(m3_inA), m3_inB(m3_inB), m3_en(m3_en), 
    m4_inA(m4_inA), m4_inB(m4_inB), m4_en(m4_en),
    m1(m1_inA, m1_inB, m1_en),
    m2(m2_inA, m2_inB, m2_en),
    m3(m3_inA, m3_inB, m3_en),
    m4(m4_inA, m4_inB, m4_en) {}

/* Moves chassis forward at the given speed. Speed is duty cycle from 0-255. */
void Drivetrain::forward(uint8_t speed) {
    m1.forward(speed); 
    m2.forward(speed);
    m3.forward(speed);
    m4.forward(speed*1.15);
}

/* Moves chassis backward at the given speed. Speed is duty cycle from 0-255. */
void Drivetrain::backward(uint8_t speed) {
    m1.backward(speed);
    m2.backward(speed);
    m3.backward(speed);
    m4.backward(speed);
}

/* Moves chassis laterally left at the given speed. Speed is duty cycle from 0-255. */
void Drivetrain::left(uint8_t speed) {
    m1.backward(speed);
    m2.forward(speed);
    m3.forward(speed);
    m4.backward(speed);
}

/* Moves chassis laterally right at the given speed. Speed is duty cycle from 0-255. */
void Drivetrain::right(uint8_t speed) {
    m1.forward(speed);
    m2.backward(speed);
    m3.backward(speed);
    m4.forward(speed);
}

/* Rotates chassis clockwise at the given speed. Speed is duty cycle from 0-255. 
Angle is in degrees from 0-360. */
void Drivetrain::rot_cw(uint8_t speed, uint8_t angle) {
    // TO DO: add angle / duration logic 

    // to implement, review Mecanum wheel movements to achieve a rotation 
    // there are various approaches to rotation we can implement/test
      // 1) duration based -> duration based rotation to achieve a specific angle 
      // 2) infinite -> infinite cw/ccw rotation until higher-level code receives some sensory input, then stop rotation

    m1.forward(speed);
    m2.backward(speed);
    m3.forward(speed);
    m4.backward(speed);
}

/* Rotates chassis counter clockwise at the given speed. Speed is duty cycle from 0-255. 
Angle is in degrees from 0-360. */
void Drivetrain::rot_ccw(uint8_t speed, uint8_t angle) {
    // TO DO: add angle / duration logic 
    m1.backward(speed);
    m2.forward(speed);
    m3.backward(speed);
    m4.forward(speed);
}

/* Stops the chassis. */
void Drivetrain::stop() {
    m1.stop();
    m2.stop();
    m3.stop();
    m4.stop();
}