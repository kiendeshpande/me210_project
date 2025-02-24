#include "motor.h"

Motor::Motor(uint8_t inA, uint8_t inB, uint8_t enable) 
    : inA(inA), inB(inB), enable(enable) {

    // set pin modes 
    pinMode(inA, OUTPUT); 
    pinMode(inB, OUTPUT); 
    pinMode(enable, OUTPUT); 
}

/* Drives single motor forward at the given speed. Speed is duty cycle from 0-255 */
void Motor::forward(uint8_t speed) {
    digitalWrite(inA, HIGH); 
    digitalWrite(inB, LOW); 
    analogWrite(enable, speed); 
}


/* Drives single motor backward at given speed. Speed is duty cycle from 0-255 */
void Motor::backward(uint8_t speed) {
    digitalWrite(inA, LOW); 
    digitalWrite(inB, HIGH); 
    analogWrite(enable, speed); 
}

/* Stops single motor */
void Motor::stop() {
    analogWrite(enable, 0); // to stop, set enable pin duty cycle to 0
}