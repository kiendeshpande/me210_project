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


// Servo pinouts
#define SERVO_DROP_PIN 2 
#define SERVO_FLAG_PIN 3

// US pinouts: 4-13
#define US_F_TRIG 5     // Front
#define US_F_ECHO 4
#define US_B_TRIG 3     // Back
#define US_B_ECHO 2
#define US_R_TRIG 7     // Right
#define US_R_ECHO 6
#define US_L1_TRIG 12   // Left 1
#define US_L1_ECHO 13
#define US_L2_TRIG 9    // Left 2
#define US_L2_ECHO 8

// IR Beacon pinout
#define IR_PIN A0
#define BEACON_THRESHOLD 3.3

#define SZ_THRESH 18
#define SZ_DIR 38


/*---------------State Definitions--------------------------*/
typedef enum {
    STATE_START, STATE_ORIENT, STATE_FWD_1, STATE_RIGHT, STATE_FWD_2, 
    STATE_INSIDE_LEFT, STATE_OUTSIDE_LEFT_1, STATE_INSIDE_BACK, 
    STATE_OUTSIDE_BACK, STATE_OUTSIDE_LEFT_2, STATE_IGNITE, STATE_FWD, STATE_DROP
  } States_t;
  

/*---------------Module Variables---------------------------*/
States_t state;
float us_f, us_b, us_r, us_l1, us_l2;
uint16_t IR_value;

bool command_sent = false; 

// Initialize objects 
//Payload p1(SERVO_DROP_PIN);
//Payload p2(SERVO_FLAG_PIN);

Ultrasonic us_front(US_F_TRIG, US_F_ECHO);
Ultrasonic us_back(US_B_TRIG, US_B_ECHO);
Ultrasonic us_right(US_R_TRIG, US_R_ECHO);
Ultrasonic us_left1(US_L1_TRIG, US_L1_ECHO);
Ultrasonic us_left2(US_L2_TRIG, US_L2_ECHO);

// i2c TX sent as a struct with required data for one drivetrain command
// this includes the task_id (forward, backward, etc.) and required arguments (speed and angle)
struct i2c_payload {
    i2c_payload(): task_id(0), speed(0), angle(0) {}
    uint8_t task_id, speed, angle; 
};
static i2c_payload tx_data; 


/*---------------State and Helper Functions---------------------------*/

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
    //p1.begin();  // initialize payload 
    
    // initialize ultrasonic 
    // TO DO: write a sensors class and handle ultrasonics within this 
    us_front.begin();  // initialize ultrasonic sensors
    us_back.begin();
    us_left1.begin();
    us_left2.begin();
    us_right.begin();
  
    Serial.begin(9600);

    state = STATE_ORIENT;
}



void loop() {
    checkGlobalEvents();
    switch (state) {
      case STATE_ORIENT:        
        handleOrient();
        break;
      default:    
        break;
    }
  }


// Handler for global events & responses
void checkGlobalEvents(void) {
    if (state == STATE_ORIENT) {
        // ping all US sensors
        us_f = us_front.distance();
        us_b = us_back.distance();
        us_r = us_right.distance();
        us_l1 = us_left1.distance();
        us_l2 = us_left2.distance();
    } else if (state == STATE_RIGHT) {
        // ping only right US sensor
        // analog read beacon sensor
        us_r = us_right.distance();
        IR_value = analogRead(IR_PIN);
        IR_value = map(IR_value, 0, 1023, 0, 5);
    } else if ((state == STATE_INSIDE_LEFT) || 
               (state == STATE_OUTSIDE_LEFT_1) || 
               (state == STATE_OUTSIDE_LEFT_2)) {
        // ping only left US sensor
        // TODO: read both or just one of left US
        us_l1 = us_left1.distance();
    } else if ((state == STATE_FWD_1) || 
               (state == STATE_FWD_2) || 
               (state == STATE_INSIDE_BACK) || 
               (state == STATE_OUTSIDE_BACK) || 
               (state == STATE_DROP)) {
        // ping only front US sensor
        us_f = us_front.distance();
    } else {
        return;
    }
  }


/* Routine for orientation state. Square the robot inside the start zone. CCW rotation until 
the two leftside ultrasonics are aligned and the front ultrasonic faces desired direction */
void handleOrient () {
    float err = 0.4; // tolerance when determining alignment between us sensors 
    bool direction = (us_f > SZ_DIR) & (us_l1 < SZ_THRESH) & (us_l2 < SZ_THRESH); 
    bool aligned   = abs(us_l1 - us_l2) < err;  

    if (direction & aligned) {
        command_slave(STOP);
        command_sent = false;
        state = STATE_FWD;
        return; // correctly oriented 
    }

    if (command_sent == false) {   // prevent repeated rotation commands 
        command_slave(ROT_CCW, 90); 
        command_sent = true; 
    }
}


