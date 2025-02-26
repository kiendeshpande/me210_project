#include <Wire.h>
#include "ultrasonic.h"

// Task IDs corresponding to drivetrain movement (used by I2C messaging)
#define FORWARD  1
#define BACKWARD 2
#define LEFT     3
#define RIGHT    4
#define ROT_CW   5
#define ROT_CCW  6
#define STOP     7

#define SLAVE_ADDR 9  

// i2c TX sent as a struct with required data for one drivetrain command
// this includes the task_id (forward, backward, etc.) and required arguments (speed and angle)
struct i2c_payload {
    i2c_payload(): task_id(0), speed(0), angle(0) {}
    uint8_t task_id, speed, angle; 
};
static i2c_payload tx_data; 


// TEST FOR ULTRASONIC 
Ultrasonic us1(8, 7);


/* Sends TX to slave given a drivetrain task id, speed, and angle. */
void command_slave(uint8_t task_id, uint8_t speed = 100, uint8_t angle = 0) {
    Wire.beginTransmission(SLAVE_ADDR);
    tx_data.task_id = task_id;
    tx_data.speed = speed;
    tx_data.angle = angle;
    Wire.write((byte*) &tx_data, sizeof(tx_data));
    Wire.endTransmission();
}


void setup() {
    Wire.begin();  // start i2c bus as master 
    Serial.begin(9600);
}

void loop() {
    // test command for ultrasonic sensor  
    float us1_dist = us1.distance();
    Serial.println(us1_dist); 
    delay(3000);

    // simple test sequence sending drivetrain commands to slave 
    Serial.println("NEW COMMAND SEQUENCE");
    command_slave(FORWARD, 100); 
    delay(5000);
    command_slave(BACKWARD, 100); 
    delay(5000);
    command_slave(LEFT, 100); 
    delay(5000);
    command_slave(RIGHT, 100); 
    delay(5000);
    command_slave(STOP, 100); 
    delay(5000);
}


// --------------------------------------

// void setup() {
//   // Gather Orientation
// }

// void loop() {
//   // dominate

// }

// void orient_self() {
//   // Turn steadily clockwise until minimum values are found
//   // Maybe turn back the other way to make sure we didn't overshoot?
// }

// void gather_pot() {
//   //Drive forward 6 inches
//   // drive right until 2 inches from right wall
//   // drive forward until full distance from wall
//   // drive left until pot is in on burner
// }

// void trigger_burner_to pantry() {
//   // back up
//   // extend appendage
//   // drive left until the wall
//   // drive right until 4 inches from the wall
//   // begin power to flywheel
//   // drive back 12 inches
// }

// void ready_aim_fire() {
//   // maybe rotate some amount? (Or built the cannon at a controllable angle)
//   // start cannon feed motor
//   // repeat until we need to finish
// }

// void return_pot() {
//   // disengage burner
//   // drag pot back
//   // boogie_time()
// }
