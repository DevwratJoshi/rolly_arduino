### Summary
This program controls the velocity of rolly's wheels in rad/s with a simple pid control algorithm

### Basic working:  
1. The arduino reads the encoder pins of rolly's motors to get the current velocity.  
2. The board listens on the serial port for messages of the form: <GVR,GVL>, where GVR and GVL are the goal rotational velocities of the right and left wheels respectively.  
3. When a command in the above format is sent to the arduino, it returns a response of the form (CVR, CVL), where CVR and CVL are the "current" rotational velocities of the right and left wheels respectively. In other words, a command needs to be sent in order to get the current speed. (Can just send the same goal wheel velocities multple times to get the wheel evolving wheel velocities if necessary)  
 

### Usage:  
1. Load the program on the arduino onboard rolly  
2. Send a command either through the serial monitor, or through the connected pi through the serial port 
