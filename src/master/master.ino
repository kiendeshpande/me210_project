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


// ultrasonic pinouts 
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

#define SZ_THRESH 25
#define SZ_DIR 38

// Servo pinouts
// #define SERVO_FLAG_PIN 10
#define SERVO_IGNITE_PIN 11
#define SERVO_DROP_PIN 10


/*---------------State Definitions--------------------------*/

/* More states to be added, enum limited to currently implemented states */
typedef enum {
    STATE_START, STATE_ORIENT, STATE_LEFT1, STATE_FWD1, STATE_IGNITE, STATE_RIGHT1, STATE_FWD2, STATE_LEFT2, STATE_RIGHT2, 
    STATE_BACK1, STATE_LOAD, STATE_FWD3, STATE_LEFT3, STATE_RIGHT3, STATE_FWD4, STATE_DROP, STATE_BACK2, STATE_DONE
  } States_t;
  

/*---------------Module Variables---------------------------*/
States_t state;
float us_f, us_b, us_r, us_l1, us_l2, us_l1l2_err;
float timer_tmp; 

bool command_sent = false;  // used to prevent repeated commands while in same state

// Initialize objects 
Ultrasonic us_front(US_F_TRIG, US_F_ECHO);
Ultrasonic us_back(US_B_TRIG, US_B_ECHO);
Ultrasonic us_right(US_R_TRIG, US_R_ECHO);
Ultrasonic us_left1(US_L1_TRIG, US_L1_ECHO);
Ultrasonic us_left2(US_L2_TRIG, US_L2_ECHO);

// i2c TX sent as a struct with required data for one drivetrain command
// this includes the task_id (forward, backward, etc.) and required arguments (speed and angle)
struct i2c_payload {
    i2c_payload(): task_id(0), speed(0) {}
    uint8_t task_id, speed;
};
static i2c_payload tx_data; 

// Initialize objects 
// Payload p_flag(SERVO_FLAG_PIN);
Payload p_ignite(SERVO_IGNITE_PIN);
Payload p_drop(SERVO_DROP_PIN);


/*---------------State and Helper Functions---------------------------*/

/* Sends TX to slave given a drivetrain task id, speed, and angle. */
void command_slave(uint8_t task_id, uint8_t speed = 100, uint8_t kp1 = 1, uint8_t kp2 = 1) {
    Wire.beginTransmission(SLAVE_ADDR);
    tx_data.task_id = task_id;
    tx_data.speed = speed;
    Wire.write((byte*) &tx_data, sizeof(tx_data));
    Wire.endTransmission();
}


Ultrasonic us_front(US_F_TRIG, US_F_ECHO);
Ultrasonic us_back(US_B_TRIG, US_B_ECHO);
Ultrasonic us_right(US_R_TRIG, US_R_ECHO);
Ultrasonic us_left1(US_L1_TRIG, US_L1_ECHO);
Ultrasonic us_left2(US_L2_TRIG, US_L2_ECHO);


/*---------------Main Functions----------------------------*/
void setup() {
    Wire.begin();  // start i2c bus as master 
    
    // TO DO: write a robot/sensors class and handle ultrasonics within this 
    us_front.begin();  
    us_back.begin();
    us_left1.begin();
    us_left2.begin();
    us_right.begin();

    p_ignite.begin();
    p_drop.begin();  
  
    Serial.begin(9600);

    // state = STATE_ORIENT; 
    state = STATE_START;
}



void loop() {
    checkGlobalEvents();
    switch (state) {
      case STATE_START:
          handle_start();
          break;
      case STATE_ORIENT:        
          handle_orient();
          break;
      case STATE_LEFT1:   // Left to wall so close to igniter after orient
          handle_left1();
          break;
      case STATE_FWD1:    // align with igniter         
          handle_fwd1();
          break;
      case STATE_IGNITE:  // hit igniter
          handle_ignite();
          break;
      case STATE_RIGHT1:   // right to far wall
          handle_right1();
          break;
      case STATE_FWD2:   // fwd to wall
          handle_fwd2();
          break;
      case STATE_LEFT2:   // push pot to burner
          handle_left2();
          break; 
/*---------------Start of states for continuous loop during round---------------------------*/
      case STATE_RIGHT2:  // right to far wall
          handle_right2(); 
          break;
      case STATE_BACK1: // backup to loading area
          handle_back1();
          break;
      case STATE_LOAD:    // load balls
          handle_load();
          break;
      case STATE_FWD3:   // fwd to clear kitchen and handles
          handle_fwd3();
          break;
      case STATE_LEFT3:   // left to far wall
          handle_left3();
          break;
      case STATE_RIGHT3:   // right a little bit to clear first handle
          handle_right3();
          break;
      case STATE_FWD4:   // fwd to drop
          handle_fwd4();
          break;
      case STATE_DROP:  // drop balls
          handle_drop();
          break; 
      case STATE_BACK2:  // back up to clear handles and transition to right2 state
          handle_back2();
          break;
      case STATE_DONE:
          handle_done();
          break;
      default:    
          break;
    }
  }


/* Handler for global events & responses */
void checkGlobalEvents(void) {
    if (state == STATE_ORIENT) {
        us_f = us_front.distance();
        us_l1 = us_left1.distance();
        us_l2 = us_left2.distance();
        us_r = us_right.distance();
        us_l1l2_err = abs(us_l1 - us_l2);
    }
}

// Initialize servos to correct positions
void handle_start() {
  p_drop.init_drop();
  p_ignite.init_ignite();
  state = STATE_ORIENT;
}

/* Routine for orientation state. Square the robot inside the start zone. Rotation until 
the two leftside ultrasonics are aligned and the front ultrasonic faces desired direction */
void handle_orient () {
    float err = 0.75; // tolerance when determining alignment between us sensors 
    bool direction = (us_f > SZ_DIR) & (us_l1 < SZ_THRESH) & (us_l2 < SZ_THRESH) & (us_r < SZ_DIR); 
    bool aligned   = us_l1l2_err < err;  

    if (direction & aligned) {
        command_slave(STOP);
        command_sent = false;
        state = STATE_LEFT1;
        delay(2000);
        return; // correctly oriented 
    }

    if (command_sent == false) {   
        command_slave(ROT_CCW, 90); 
        command_sent = true; 
    }
    return;
}

/* Push pot left across the course, onto the burner */
void handle_left1() {
  if (command_sent == false) {
    command_slave(FORWARD, 140);
    timer_tmp = millis();
    command_sent = true;
}

float elapsed = millis() - timer_tmp; 
if (elapsed < 200)
  return;

command_slave(STOP);
command_sent = false;
state = STATE_FWD1;
return;
}


/* Forward state to exit start zone and position next to igniter */
void handle_fwd1() {
    if (command_sent == false) {
        // note the drivetrain movements conform to the prior direction convention
        // this is where "forward" is the direction with the 2 ultrasonic sensors on one face
        // TO DO: once beat-the-brick complete, fix nomenclature so drivetrain directions match convention in this file
        command_slave(RIGHT, 140);  
        command_sent = true;
    }

    if (us_f > 80) 
      return; 

    command_slave(STOP);
    command_sent = false;
    state = STATE_IGNITE;
    return;
}


void handle_ignite() {
  command_slave(STOP);
  delay(1000);
  p_ignite.ignite();
  delay(1000);
  state = STATE_RIGHT1;
}


/* Travel from left side of field to right side of field using elapsed time */
void handle_right1() {
    if (command_sent == false) {
      command_slave(BACKWARD, 140);
      timer_tmp = millis(); // record current time
      command_sent = true;
    }

    float elapsed = millis() - timer_tmp;  // elapsed time in this state
    if (elapsed < 2300) 
      return; 

    command_slave(STOP);
    command_sent = false;
    state = STATE_FWD2; 
    delay(1000);
    return;
}

/* Align robot with far wall prior to pushing the pot */
void handle_fwd2() {
  if (command_sent == false) {
      command_slave(RIGHT, 140);
      timer_tmp = millis();
      command_sent = true;
  }

  float elapsed = millis() - timer_tmp; 
  if (elapsed < 1200) 
    return; 

  command_slave(STOP);
  command_sent = false;
  state = STATE_LEFT2; 
  return;
}

/* Push pot left across the course, onto the burner */
void handle_left2() {
    if (command_sent == false) {
      command_slave(FORWARD, 140);
      timer_tmp = millis();
      command_sent = true;
  }

  float elapsed = millis() - timer_tmp; 
  if (elapsed < 3000)
    return;

  command_slave(STOP);
  command_sent = false;
  state = STATE_RIGHT2;
  return;
}

//-------------------------------- start of loop states----------------------------------//

// go back to right wall
void handle_right2() {
  if (command_sent == false) {
    command_slave(BACKWARD, 140);
    timer_tmp = millis(); // record current time
    command_sent = true;
  }

  float elapsed = millis() - timer_tmp;  // elapsed time in this state
  if (elapsed < 2000) 
    return; 

  command_slave(STOP);
  command_sent = false;
  state = STATE_BACK1; 
  delay(1000);
  return;
}

void handle_back1() {
    if (command_sent == false) {
      command_slave(LEFT, 140);
      timer_tmp = millis();
      command_sent = true;
  }

  float elapsed = millis() - timer_tmp; 
  if (elapsed < 1400)
    return;

  command_slave(STOP);
  command_sent = false;
  state = STATE_LOAD;
  return;
}


void handle_load() {
  delay(2000);
  state = STATE_FWD3; 
  return;
}


void handle_fwd3() {
  if (command_sent == false) {
    command_slave(RIGHT, 140);
    timer_tmp = millis();
    command_sent = true;
  }

  float elapsed = millis() - timer_tmp; 
  if (elapsed < 1200)
    return;

  command_slave(STOP);
  command_sent = false;
  state = STATE_LEFT3;
  return;
}


void handle_left3() {
    if (command_sent == false) {
      command_slave(FORWARD, 140);
      timer_tmp = millis();
      command_sent = true;
  }

  float elapsed = millis() - timer_tmp; 
  if (elapsed < 3000)
    return;

  command_slave(STOP);
  command_sent = false;
  state = STATE_RIGHT3;
  return;
}


void handle_right3() {
    if (command_sent == false) {
      command_slave(BACKWARD, 140);
      timer_tmp = millis(); // record current time
      command_sent = true;
    }

    float elapsed = millis() - timer_tmp;  // elapsed time in this state
    if (elapsed < 170) 
      return; 

    command_slave(STOP);
    command_sent = false;
    state = STATE_FWD4; 
    // delay(1000);
    return;
}


void handle_fwd4() {
  if (command_sent == false) {
    command_slave(RIGHT, 140);
    timer_tmp = millis();
    command_sent = true;
}

float elapsed = millis() - timer_tmp; 
if (elapsed < 1200)
  return;

command_slave(STOP);
command_sent = false;
state = STATE_DROP;
return;
}


void handle_drop() {
  delay(1000);
  p_drop.release();
  delay(1000);
  state = STATE_BACK2;
}


void handle_back2() {
  if (command_sent == false) {
    command_slave(LEFT, 140);
    timer_tmp = millis();
    command_sent = true;
  }

  float elapsed = millis() - timer_tmp; 
  if (elapsed < 1400)
    return;

  command_slave(STOP);
  command_sent = false;
  state = STATE_RIGHT2;
  return;
}


void handle_done() {
  state = STATE_DONE;
}

// Handler for when bot is moving fwd out of start zone 
void handleFwd1(void) {
    // Move fwd
    if (!command_sent) {
        command_slave(LEFT, 120); 
        command_sent = true;
    }
    // Check distance
    if (us_f < 10) {       // TODO: calibrate distance on course
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
    if (us_r < 5) {       // TODO: calibrate distance on course
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