#include <Wire.h>
#include "ultrasonic.h"
#include "payload.h"

// Task IDs corresponding to drivetrain movement (used by I2C messaging)
#define FORWARD  1
#define BACKWARD 2
#define LEFT     3
#define RIGHT    4      // TODO: change to reflect new forward frame of reference
#define ROT_CW   5  
#define ROT_CCW  6  
#define STOP     7

#define SLAVE_ADDR 9 

// Servo pinouts
#define SERVO_FLAG_PIN 1
#define SERVO_IGNITE_PIN 2
#define SERVO_DROP_PIN 3

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

#define SZ_THRESH 18
#define SZ_DIR 38

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
float us_f, us_b, us_r, us_l1, us_l2;   // Distance variables
uint16_t IR_value;

bool beacon_sensed = false;     // beacon sensed flag
bool command_sent = false;      // i2c command flag

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
    // state = STATE_START; // Initial state
    state = STATE_ORIENT;  
    startMillis = millis(); // Start time
}

// Test individual functions by commenting out cases to be left out
void loop() {
    checkGlobalEvents();
    switch (state) {
    //   case STATE_START:             // Start of round
    //     handleStart();
    //     break;
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
// Poll sensors based off of current state
void checkGlobalEvents(void) {
    currentMillis = millis();
    if (currentMillis - startMillis >= 130000) {    // 130 second rounds
        state == STATE_ROUND_OVER;                  // TODO: Blocking code may effect round end timing
    }
    if (state == STATE_ORIENT) {
        // ping all US sensors
        us_f = us_front.distance();
        us_b = us_back.distance();
        us_r = us_right.distance();
        us_l1= us_left1.distance();
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


// Handler for start state
void handleStart(void) {
    // TODO: Implement servo action
    // Blocking code is probably fine here

    // p_flag.release(); // TODO: configure servo to lower flag
    state = STATE_FWD_1;
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
        state = STATE_FWD_1;
        return; // correctly oriented 
    }

    if (command_sent == false) {   // prevent repeated rotation commands 
        command_slave(ROT_CCW, 90); 
        command_sent = true; 
    }
}


// Handler for when bot is moving fwd out of start zone 
void handleFwd1(void) {
    // Move fwd
    if (!command_sent) {
        command_slave(LEFT, 120); 
        command_sent = true;
    }
    // Check distance
    if (us_f < 30) {       // TODO: calibrate distance on course
        state = STATE_RIGHT;
        command_slave(STOP);
    }
}


// Handler for when bot is moving right along the kitchen 
// and searching for right wall or IR beacon
void handleRight(void) {
    // Move right
    if (!command_sent) {
        command_slave(RIGHT, 180); 
        command_sent = true;
    }
    // Check distance
    if (us_r < 13) {       // TODO: calibrate distance on course
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
    if (!command_sent) {
        command_slave(FORWARD, 180); 
        command_sent = true;
    }
    // Check distance
    if (us_f < 7) {       // TODO: calibrate distance on course
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
    if (!command_sent) {
        command_slave(LEFT, 180); 
        command_sent = true;
    }
    // Check distance
    if (us_l1 < 7) {        // TODO: calibrate distance on course
        state = STATE_INSIDE_BACK;
        command_slave(STOP);
    }
}  


// Handler for when bot is pushing pot OUTSIDE the handles,
// sensing left wall will be further than when inside handles
void handleOutsideLeft1(void) {
    // Move left
    if (!command_sent) {
        command_slave(LEFT, 180); 
        command_sent = true;
    }
    // Check distance
    if (us_l1 < 13) {       // TODO: calibrate distance on course
        state = STATE_OUTSIDE_BACK;
        command_slave(STOP);
    }
} 


// Handler for when bot is backing out of handles
void handleInsideBack(void) {
    // Move backwards
    if (!command_sent) {
        command_slave(BACKWARD, 180); 
        command_sent = true;
    }
    // Check distance
    if (us_f > 13) {        // TODO: calibrate distance on course
        state = STATE_IGNITE;
        command_slave(STOP);
    }
}  


// Handler for when bot is backing up when outside of handles
void handleOutsideBack(void) {
    // Move backwards
    if (!command_sent) {
        command_slave(BACKWARD, 180); 
        command_sent = true;
    }
    // Check distance
    if (us_f > 13) {        // TODO: calibrate distance on course
        state = STATE_OUTSIDE_LEFT_2;
        command_slave(STOP);
    }
}  


// Handler for when bot is going left to ignite
void handleOutsideLeft2(void) {
    // Move left
    if (!command_sent) {
        command_slave(LEFT, 180); 
        command_sent = true;
    }
    // Check distance
    if (us_l1 < 7) {       // TODO: calibrate distance on course
        state = STATE_IGNITE;
        command_slave(STOP);
    }
} 


// Handler for pushing ignite button
void handleIgnite(void) {
    // TODO: Implement servo action
    // Blocking code is probably fine here
    p_ignite.release(); // TODO: configure servo to hit igniter
    // TODO: add state transition
} 


// Handler for when bot is moving fwd to drop ball into pot
void handleFwd3(void) {
    // Move forward
    if (!command_sent) {
        command_slave(FORWARD, 180); 
        command_sent = true;
    }
    // Check distance
    if (us_f < 7) {       // TODO: calibrate distance on course
        state = STATE_DROP;
        command_slave(STOP);
    }
}


// Handler for dropping ingredient into pot
void handleDrop(void) {
    // TODO: Implement servo action
    // Blocking code is probably fine here
    p_drop.release(); // TODO: configure servo to release ingredient
    // TODO: add state transition
}


// Handler for round over
void handleRoundOver(void) {
    // TODO: Implement servo action
    // Blocking code is probably fine here
    command_slave(STOP); 
    p_flag.release();   // TODO: servo to raise flag for round end
}