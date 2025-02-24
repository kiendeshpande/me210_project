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

Drivetrain chassis(M1_INA, M1_INB, M1_EN,
                   M2_INA, M2_INB, M2_EN,
                   M3_INA, M3_INB, M3_EN,
                   M4_INA, M4_INB, M4_EN); 


void setup() {
  Serial.begin(9600); 
  while (!Serial) {}; 
}

void loop() 
{
  // test sequence to test chassis/drivetrain functionality 
  // the untested feature is cw/ccw rotation -> requires assembled chassis to implement and test 
  // 100 is chosen as arbitrary speed (duty cycle range is 0-255)
  chassis.forward(100);
  delay(5000);
  chassis.backward(100);
  delay(5000);
  chassis.left(100);
  delay(5000);
  chassis.right(100);
  delay(5000);
  chassis.stop();
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
