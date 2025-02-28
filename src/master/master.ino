#include <Wire.h>
#include "ultrasonic.h"
#include "payload.h"

// Task IDs corresponding to drivetrain movement (used by I2C messaging)
#define FORWARD  1
#define BACKWARD 2
#define LEFT     3
#define RIGHT    4
#define ROT_CW   5
#define ROT_CCW  6
#define STOP     7

#define SLAVE_ADDR 9  

#define SERVO_PIN 8

// i2c TX sent as a struct with required data for one drivetrain command
// this includes the task_id (forward, backward, etc.) and required arguments (speed and angle)
struct i2c_payload {
    i2c_payload(): task_id(0), speed(0), angle(0) {}
    uint8_t task_id, speed, angle; 
};
static i2c_payload tx_data; 


/* Sends TX to slave given a drivetrain task id, speed, and angle. */
void command_slave(uint8_t task_id, uint8_t speed = 100, uint8_t angle = 0) {
    Wire.beginTransmission(SLAVE_ADDR);
    tx_data.task_id = task_id;
    tx_data.speed = speed;
    tx_data.angle = angle;
    Wire.write((byte*) &tx_data, sizeof(tx_data));
    Wire.endTransmission();
}

// Initialize objects 
Payload p1(SERVO_PIN);

void setup() {
    Wire.begin();  // start i2c bus as master 
    p1.begin();  // initialize payload 

    Serial.begin(9600);
}

void loop() {
    // // test command for ultrasonic sensor  
    // // float us1_dist = us1.distance();
    // // Serial.println(us1_dist); 
    // // delay(3000);
    // p1.release(0);
    // delay(1000);
    // p1.release(180);
    // delay(1000);


    // simple test sequence sending drivetrain commands to slave 
    Serial.println("NEW COMMAND SEQUENCE");
    command_slave(FORWARD, 180); 
    delay(3000);
    command_slave(STOP, 180); 
    delay(2000);
    command_slave(BACKWARD, 180); 
    delay(3000);
    command_slave(STOP, 180); 
    delay(2000);
    command_slave(LEFT, 180); 
    delay(3000);
    command_slave(STOP, 180); 
    delay(2000);
    command_slave(RIGHT, 180); 
    delay(3000);
    command_slave(STOP, 180); 
    delay(2000);
}


