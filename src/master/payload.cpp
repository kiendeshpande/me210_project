#include "payload.h"

Payload::Payload(uint8_t servo_pin) : servo_pin(servo_pin) {} 

/* Initialize payload pins */
void Payload::begin() {
  servo.attach(servo_pin);
}

// /* Opens and closes servo to release ingredients from payload. */
// void Payload::release() {
//     servo.write(20); 
//     delay(2000); 
//     servo.write(135); 
// }

/* Opens and closes servo to release ingredients from payload. */
void Payload::ignite() {
    servo.write(180); 
    delay(1000); 
    servo.write(70); 
}

void Payload::init_ignite() {
    servo.write(70); 
}

// Start/stop flag servo functions
void Payload::raise_flag() {
    servo.write(30); 
}

void Payload::lower_flag() {
    servo.write(120); 
}