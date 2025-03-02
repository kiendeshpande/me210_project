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
#define US_R_TRIG 6
#define US_R_ECHO 7
#define IR_PIN A0

#define BEACON_THRESHOLD 3.3

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


/*---------------State Definitions--------------------------*/
typedef enum {
    STATE_START, STATE_ORIENT, STATE_FWD_1, STATE_RIGHT, STATE_FWD_2, 
    STATE_INSIDE_LEFT, STATE_OUTSIDE_LEFT_1, STATE_INSIDE_BACK, 
    STATE_OUTSIDE_BACK, STATE_OUTSIDE_LEFT_2, STATE_IGNITE, STATE_FWD, STATE_DROP
  } States_t;
  
/*---------------Module Variables---------------------------*/
States_t state;
float us_left_dist, us_right_dist, us_back_dist, us_front_dist;
uint16_t IR_value;
uint16_t IR_mapped;

// Initialize objects 
Payload p1(SERVO_PIN);
Ultrasonic us_right(US_R_TRIG, US_R_ECHO);


/*---------------Main Functions----------------------------*/
void setup() {
    Wire.begin();  // start i2c bus as master 
    p1.begin();  // initialize payload 
    us_front.begin();  // initialize ultrasonic sensors
    us_back.begin();
    us_left.begin();
    us_right.begin();
    Serial.begin(9600);
}


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
        us_left_dist = us_left.distance();
        us_right_dist = us_right.distance();
        us_front_dist = us_front.distance();
        us_back_dist = us_back.distance();
    } else if (state == STATE_RIGHT) {
        // ping only right US sensor
        // analog read beacon sensor
        us_right_dist = us_right.distance();
        IR_value = analogRead(IR_PIN);
        IR_mapped = map(IR_value, 0, 1023, 0, 5);
    } else if ((state == STATE_INSIDE_LEFT) || 
               (state == STATE_OUTSIDE_LEFT_1) || 
               (state == STATE_OUTSIDE_LEFT_2)) {
        // ping only left US sensor
        us_left_dist = us_left.distance();
    } else if ((state == STATE_FWD_1) || 
               (state == STATE_FWD_2) || 
               (state == STATE_INSIDE_BACK) || 
               (state == STATE_OUTSIDE_BACK) || 
               (state == STATE_DROP)) {
        // ping only front US sensor
        us_front_dist = us_front.distance();
    } else {
        break;
    }
  }


void handleRight(void) {
    // move right
    if ((us_right_dist < 13) || (IR_mapped > BEACON_THRESHOLD)) {
        state = STATE_FWD_2;
    }
}   
