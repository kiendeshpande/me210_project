#include "payload.h"

Payload::Payload(uint8_t servo_pin) : servo_pin(servo_pin) {} 

/* Initialize payload pins */
void Payload::begin() {
  servo.attach(servo_pin);
}

/* Opens and closes servo to release ingredients from payload. */
void Payload::release() {
    servo.write(90); 
    delay(2000); 
    servo.write(0); 
}