#include <Wire.h>
#include "ultrasonic.h"
#include "payload.h"

// Task IDs corresponding to drivetrain movement (used by I2C messaging)
#define FORWARD  1
#define BACKWARD 2
#define LEFT     3
#define RIGHT    4
#define ROT_CW   5  // TO DO: implement rotation in drivetrain.cpp / .h
#define ROT_CCW  6  // TO DO: implement rotation in drivetrain.cpp / .h
#define STOP     7

#define SLAVE_ADDR 9 

// Servo pinouts
#define SERVO_FLAG_PIN 1
#define SERVO_IGNITE_PIN 2
#define SERVO_DROP_PIN 3

// US pinouts: 4-13
#define US_F_TRIG 4     // Front
#define US_F_ECHO 5

#define US_B_TRIG 6     // Back
#define US_B_ECHO 7

#define US_R_TRIG 8     // Right
#define US_R_ECHO 9

#define US_L1_TRIG 10   // Left 1
#define US_L1_ECHO 11

#define US_L2_TRIG 12   // Left 2
#define US_L2_ECHO 13

// IR Beacon pinout
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


/*---------------Module Function Prototypes-----------------*/
void checkGlobalEvents(void);
void handleStart(void);
void handleOrient(void);
void handleFwd1(void);
void handleRight(void);
void handleFwd2(void);
void handleInsideLeft(void);
void handleOutsideLeft1(void);
void handleInsideBack(void);
void handleOutsideBack(void);
void handleOutsideLeft2(void);
void handleIgnite(void);
void handleFwd3(void);
void handleDrop(void);
void handleRoundOver(void);


/*---------------State Definitions--------------------------*/
typedef enum {
    STATE_START, STATE_ORIENT, STATE_FWD_1, STATE_RIGHT, STATE_FWD_2, 
    STATE_INSIDE_LEFT, STATE_OUTSIDE_LEFT_1, STATE_INSIDE_BACK, STATE_OUTSIDE_BACK, 
    STATE_OUTSIDE_LEFT_2, STATE_IGNITE, STATE_FWD_3, STATE_DROP, STATE_ROUND_OVER
  } States_t;
  
/*---------------Module Variables---------------------------*/
States_t state;
float us_front_dist, us_back_dist, us_right_dist, us_left1_dist, us_left2_dist;
uint16_t IR_value;

bool beacon_sensed = false;
bool command_right_sent = false;
bool command_fwd1_sent = false;
bool command_fwd2_sent = false;
bool command_inside_left_sent = false;
bool command_outside_left1_sent = false;
bool command_inside_back_sent = false;
bool command_outside_back_sent = false;
bool command_outside_left2_sent = false;
bool command_fwd3_sent = false;

unsigned long startMillis;
unsigned long currentMillis;

// Initialize objects 
Payload p_flag(SERVO_FLAG_PIN);
Payload p_ignite(SERVO_IGNITE_PIN);
Payload p_drop(SERVO_DROP_PIN);


Ultrasonic us_front(US_F_TRIG, US_F_ECHO);
Ultrasonic us_back(US_B_TRIG, US_B_ECHO);
Ultrasonic us_right(US_R_TRIG, US_R_ECHO);
Ultrasonic us_left1(US_L1_TRIG, US_L1_ECHO);
Ultrasonic us_left2(US_L2_TRIG, US_L2_ECHO);


/*---------------Main Functions----------------------------*/
void setup() {
    Wire.begin();  // start i2c bus as master 
    p_flag.begin();  // initialize servo payloads 
    p_ignite.begin();
    p_drop.begin();  
    us_front.begin();  // initialize ultrasonic sensors
    us_back.begin();
    us_left1.begin();
    us_left2.begin();
    us_right.begin();
    Serial.begin(9600);
    state = STATE_START; // Initial state
    startMillis = millis(); // Start time
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
      case STATE_FWD_3:             // Forward to drop ingredient 
        handleFwd3();
        break;
      case STATE_DROP:              // Drop ingredient
        handleDrop();
        break;
      case STATE_ROUND_OVER:        // Raise flag
        handleRoundOver();
        break;
      default:    // Should never get into an unhandled state
        Serial.println("What is this I do not even...");
    }
  }


// Handler for global events & responses
void checkGlobalEvents(void) {
    currentMillis = millis();
    if (currentMillis - startMillis >= 130000) {    // 130 second rounds
        state == STATE_ROUND_OVER;
    }
    if (state == STATE_ORIENT) {
        // ping all US sensors
        us_front_dist = us_front.distance();
        us_back_dist = us_back.distance();
        us_right_dist = us_right.distance();
        us_left1_dist = us_left1.distance();
        us_left2_dist = us_left2.distance();
    } else if (state == STATE_RIGHT) {
        // ping only right US sensor
        // analog read beacon sensor
        us_right_dist = us_right.distance();
        IR_value = analogRead(IR_PIN);
        IR_value = map(IR_value, 0, 1023, 0, 5);
    } else if ((state == STATE_INSIDE_LEFT) || 
               (state == STATE_OUTSIDE_LEFT_1) || 
               (state == STATE_OUTSIDE_LEFT_2)) {
        // ping only left US sensor
        // TODO: read both or just one of left US
        us_left1_dist = us_left1.distance();
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


// Handler for start state
void handleStart(void) {
    // TODO: Implement servo action
    // Blocking code is probably fine here
    p_flag.release(); // TODO: configure servo to lower flag
}


// Handler for orient state
void handleOrient(void) {
    // TODO: implement kien's code
}


// Handler for when bot is moving fwd out of start zone 
void handleFwd1(void) {
    // Move fwd
    if (!command_fwd1_sent) {
        command_slave(FORWARD, 180); 
        command_fwd1_sent = true;
    }
    // Check distance
    if (us_front_dist < 13) {       // TODO: calibrate distance on course
        state = STATE_RIGHT;
        command_slave(STOP);
    }
}


// Handler for when bot is moving right along the kitchen 
// and searching for right wall or IR beacon
void handleRight(void) {
    // Move right
    if (!command_right_sent) {
        command_slave(RIGHT, 180); 
        command_right_sent = true;
    }
    // Check distance
    if (us_right_dist < 13) {       // TODO: calibrate distance on course
        state = STATE_FWD_2;
        command_slave(STOP);
    } 
    // Check beacon
    if (IR_value > BEACON_THRESHOLD) {
        beacon_sensed = true;
        state = STATE_FWD_2;
        command_slave(STOP);  
    }
}   


// Handler for when bot is moving fwd to push pot
void handleFwd2(void) {
    // Move forward
    if (!command_fwd2_sent) {
        command_slave(FORWARD, 180); 
        command_fwd2_sent = true;
    }
    // Check distance
    if (us_front_dist < 7) {       // TODO: calibrate distance on course
        if (beacon_sensed) {
            state = STATE_INSIDE_LEFT;
            beacon_sensed = false;
        } else {
            state = STATE_OUTSIDE_LEFT_1;
        }
        command_slave(STOP);
    }
}


// Handler for when bot is pushing pot INSIDE the handles,
// sensing left wall will be closer than when outside handles
void handleInsideLeft(void) {
    // Move left
    if (!command_inside_left_sent) {
        command_slave(LEFT, 180); 
        command_inside_left_sent = true;
    }
    // Check distance
    if (us_left1_dist < 7) {        // TODO: calibrate distance on course
        state = STATE_INSIDE_BACK;
        command_slave(STOP);
    }
}  


// Handler for when bot is pushing pot OUTSIDE the handles,
// sensing left wall will be further than when inside handles
void handleOutsideLeft1(void) {
    // Move left
    if (!command_outside_left1_sent) {
        command_slave(LEFT, 180); 
        command_outside_left1_sent = true;
    }
    // Check distance
    if (us_left1_dist < 13) {       // TODO: calibrate distance on course
        state = STATE_OUTSIDE_BACK;
        command_slave(STOP);
    }
} 


// Handler for when bot is backing out of handles
void handleInsideBack(void) {
    // Move backwards
    if (!command_inside_back_sent) {
        command_slave(BACKWARD, 180); 
        command_inside_back_sent = true;
    }
    // Check distance
    if (us_front_dist > 13) {        // TODO: calibrate distance on course
        state = STATE_IGNITE;
        command_slave(STOP);
    }
}  


// Handler for when bot is backing up when outside of handles
void handleOutsideBack(void) {
    // Move backwards
    if (!command_outside_back_sent) {
        command_slave(BACKWARD, 180); 
        command_outside_back_sent = true;
    }
    // Check distance
    if (us_front_dist > 13) {        // TODO: calibrate distance on course
        state = STATE_OUTSIDE_LEFT_2;
        command_slave(STOP);
    }
}  


// Handler for when bot is going left to ignite
void handleOutsideLeft2(void) {
    // Move left
    if (!command_outside_left2_sent) {
        command_slave(LEFT, 180); 
        command_outside_left2_sent = true;
    }
    // Check distance
    if (us_left1_dist < 7) {       // TODO: calibrate distance on course
        state = STATE_IGNITE;
        command_slave(STOP);
    }
} 


// Handler for pushing ignite button
void handleIgnite(void) {
    // TODO: Implement servo action
    // Blocking code is probably fine here
    p_ignite.release(); // TODO: configure servo to hit igniter
} 


// Handler for when bot is moving fwd to drop ball into pot
void handleFwd3(void) {
    // Move forward
    if (!command_fwd3_sent) {
        command_slave(FORWARD, 180); 
        command_fwd3_sent = true;
    }
    // Check distance
    if (us_front_dist < 7) {       // TODO: calibrate distance on course
        state = STATE_DROP;
        command_slave(STOP);
    }
}


// Handler for dropping ingredient into pot
void handleDrop(void) {
    // TODO: Implement servo action
    // Blocking code is probably fine here
    p_drop.release(); // TODO: configure servo to release ingredient
}


// Handler for round over
void handleRoundOver(void) {
    // TODO: Implement servo action
    // Blocking code is probably fine here
    command_slave(STOP); 
    p_flag.release();   // TODO: servo to raise flag for round end
}