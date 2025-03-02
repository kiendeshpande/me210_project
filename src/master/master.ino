#include <Wire.h>
#include "ultrasonic.h"
#include "payload.h"

// Task IDs corresponding to drivetrain movement (used by I2C messaging)
#define FORWARD  1
#define BACKWARD 2
#define LEFT     3
#define RIGHT    4
#define ROT_CW   5  // TO DO: implement rotation in drivetrain.cpp / .h
#define ROT_CCW  6. // TO DO: implement rotation in drivetrain.cpp / .h
#define STOP     7

#define SLAVE_ADDR 9 

#define SERVO_PIN 8 
#define US1_TRIG 6
#define US1_ECHO 7

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
Ultrasonic us1(US1_TRIG, US1_ECHO);

void setup() {
    Wire.begin();  // start i2c bus as master 
    p1.begin();  // initialize payload 
    us1.begin();  // initialize us1 

    Serial.begin(9600);
}

void loop() {
    // simple test for ultrasonic sensor  
    float us1_dist = us1.distance();
    Serial.println("US1 distance in cm is:");
    Serial.println(us1_dist); 
    delay(1000);

    // simple test servo release (open and close)
    p1.release();
    delay(1000);

    // simple test sequence sending drivetrain commands to slave 
    command_slave(FORWARD, 180); 
    delay(3000);
    command_slave(STOP); 
    delay(2000);
    command_slave(BACKWARD, 180); 
    delay(3000);
    command_slave(STOP); 
    delay(2000);
    command_slave(LEFT, 180); 
    delay(3000);
    command_slave(STOP); 
    delay(2000);
    command_slave(RIGHT, 180); 
    delay(3000);
    command_slave(STOP); 
    delay(2000);
}







/*---------------State Definitions--------------------------*/
typedef enum {
    STATE_START, STATE_ORIENT, STATE_FWD_1, STATE_RIGHT, STATE_FWD_2, 
    STATE_INSIDE_LEFT, STATE_OUTSIDE_LEFT_1, STATE_INSIDE_BACK, 
    STATE_OUTSIDE_BACK, STATE_OUTSIDE_LEFT_2, STATE_IGNITE, STATE_FWD, STATE_DROP
  } States_t;
  
  /*---------------Module Variables---------------------------*/
  States_t state;

void loop() {
    checkGlobalEvents();
    switch (state) {
      case STATE_START:             // Start of round
        handleStart();
        break;
      case STATE_ORIENT:            // Orientation
        handleOrient();
        break;
      case STATE_FWD_1:             // Forward to clear kitchen
        handleFwd1();
        break;
      case STATE_RIGHT:             // Right until pot or wall
        handleRight();
        break;
      case STATE_FWD_2:             // FWD to push pot
        handleFwd2();
        break;
      case STATE_INSIDE_LEFT:       // Move left and push pot when inside handles 
        handleInsideLeft();
        break;
      case STATE_OUTSIDE_LEFT_1:    // Move left and push pot when outside handles
        handleOutsideLeft1();
        break;
      case STATE_INSIDE_BACK:       // Back out of pot to ignite
        handleInsideBack();
        break;
      case STATE_OUTSIDE_BACK:      // Back out of pushing handles
        handleOutsideBack();
        break;
      case STATE_OUTSIDE_LEFT_2:    // Left until able to ignite
        handleOutsideLeft2();
        break;
      case STATE_IGNITE:            // Hit ignite button
        handleIgnite();
        break;
      case STATE_FWD:               // Forward to drop ingredient 
        handleFwd();
        break;
      case STATE_DROP:              // Drop ingredient
        handleDrop();
        break;
      default:    // Should never get into an unhandled state
        Serial.println("What is this I do not even...");
    }
  }




// Handler for global events & responses
void checkGlobalEvents(void) {
    if (state == STATE_ORIENT) {
        // ping all US sensors
    } else if (state == STATE_RIGHT) {
        // ping only right US sensor
        // analog read beacon sensor
        if ((us_right.distance() < 13) || (analogRead(IR_PIN) < BEACON_THRESHOLD)) {
            state = STATE_FWD_2;
            break;
        }
    } else if ((state == STATE_INSIDE_LEFT) || 
               (state == STATE_OUTSIDE_LEFT_1) || 
               (state == STATE_OUTSIDE_LEFT_2)) {
        // ping only left US sensor
    } else if ((state == STATE_FWD_1) || 
               (state == STATE_FWD_2) || 
               (state == STATE_INSIDE_BACK) || 
               (state == STATE_OUTSIDE_BACK) || 
               (state == STATE_DROP)) {
        // ping only front US sensor
    } else {
        break;
    }
  }


void handleRight(void) {
    // move right
    // if (IR beacon sensed) OR (right US < 5 inches)
        // state = STATE_FWD_2
}
