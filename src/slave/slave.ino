#include <Wire.h>
#include "drivetrain.h"

// pin instantiation below reflects test wiring (not Chris' schematic)
// this will be changed during integration
#define M1_INA 7
#define M1_INB 8
#define M1_EN 11

#define M2_INA 12
#define M2_INB 13
#define M2_EN 10 

#define M3_INA 2
#define M3_INB 4
#define M3_EN 3

#define M4_INA 6
#define M4_INB 9
#define M4_EN 5

// Task IDs corresponding to drivetrain movement (used by I2C messaging)
#define FORWARD  1
#define BACKWARD 2
#define LEFT     3
#define RIGHT    4
#define ROT_CW   5
#define ROT_CCW  6
#define STOP     7

#define I2C_SLAVE_ADDR 9


struct i2c_payload {
    i2c_payload(): task_id(0), speed(0), angle(0) {}
    uint8_t task_id, speed, angle; 
};
static i2c_payload rx_data; 

Drivetrain chassis(M1_INA, M1_INB, M1_EN,
                   M2_INA, M2_INB, M2_EN,
                   M3_INA, M3_INB, M3_EN,
                   M4_INA, M4_INB, M4_EN); 


void setup() {
    Wire.begin(I2C_SLAVE_ADDR);
    Wire.onReceive(receiveEvent); 
    Serial.begin(9600);
}

// no need for code in loop, I2C calls receiveEvent (handler function)
void loop() {}

/* Execute a drivetrain task using the task_id in rx_data struct. Function 
called each time a new command is received. */
void process_task() {
    switch (rx_data.task_id) {
        case FORWARD:
            chassis.forward(rx_data.speed);
            break;
        case BACKWARD:
            chassis.backward(rx_data.speed);
            break;
        case LEFT: 
            chassis.left(rx_data.speed);
            break;
        case RIGHT: 
            chassis.right(rx_data.speed);
            break;
        case ROT_CW: 
            chassis.rot_cw(rx_data.speed, rx_data.angle);
            break;
        case ROT_CCW:
            chassis.rot_ccw(rx_data.speed, rx_data.angle);
            break;
        case STOP:
            chassis.stop();
            break;
        default:
            Serial.println("INVALID TASK ID");
            break;
    }
}

void receiveEvent(int num_bytes) {
    if (num_bytes == sizeof(i2c_payload)) {
        byte* ptr = (byte*) &rx_data; // update rx_data struct with new information 
        for (int i = 0; i < sizeof(i2c_payload); i++) {
          ptr[i] = Wire.read();
          Serial.println(ptr[i]);
        }
        process_task(); // execute task 
    }
}




