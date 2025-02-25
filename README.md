# me210_project
Stanford ME210 Final Project 

## Software (incomplete documentation)  
### I2C Structure 
We use two Arduino Unos because the robot has four motors and two  
H-bridges. There are insufficient I/O pins on a single Uno. We use the  
Arduino Wire library for I2C communication. The `master` folder contains  
the code for the main Arduino, which processes FSM-level logic and sensory  
inputs. Based on these inputs, master Arduino commands the slave.  

The slave is only responsible for drivetrain/motor control based on received  
messages. I2C messages are sent as `i2c_payload` structs containing `task_id`,  
`speed`, and `angle`. The slave executes the drivetrain movement corresponding  
to the given `task_id`. 

