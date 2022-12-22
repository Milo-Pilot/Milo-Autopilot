# Source file of the Milo autopilot.
Philippe Dufossé 2022


## Software and hardware

Runs on a arduino-compatible nano board.
It may be edited using the arduino IDE (the arduino programming platform).
It is connected to a Bosch-sensortec BMX055 IMU (Inertial Measurement Unit)
It is also connected to a h-bridge for controling the DC motor of the autopilot.

It uses the FaBo9Axis_BMX055 library to communicate with the BMX055 IMU. This libray has been slightly modified in order to
maximize the accuracy and stability of the IMU, and renamed to "FaBo9Axis_BMX055_Milo"

The IMU is mounted on a gimbal. So all calculations are made directly in 2D, in the horizontal plane.
So this program uses only the x and y components of the IMU magnetometer, and only the z component of the IMU gyrometer; 
it does not use the accelerometer.

(Note:  the gimbal solution was chosen for the following reason: attaching the IMU directly to the boat (without gimbal), and then doing
all calculations in 3D using some fusion algorithms is apparently not possible, because it would require to calibrate the IMU
along its 3 axis x, y, z. Executing calibration movements in space with the electronic device itself only is not sufficient; they must be
done with the whole boat. These calibration movements of the whole boat can be done only in 2D, by executing two circles.)


## User interface

### On/off switch

This program is run at power-up (when this switch is set to "on")

### Pilot engagement

it automatically sets the "engaging switch" to "on".

### Calibration switch

- set to "on" : pilot set to calibration mode.
- set to "off" pilot set to normal operation mode.

### gain potentiometer

It affects width of the deadband of the pilot. It reduces or increases this deadband.


## What this program do : 

### If the calibration switch is "on":

  User is requested to execute 2 circles with the boat, while doing this, the program computes the calibration offsets
  along x and y axis of the IMU. When this is done these offsets are stored in the EEPROM of the board, for further use.

### If the calibration switch is "off" :
  
  The program constantly computes the current heading angle and current angular speed of the boat.
  When user engages the pilot, a sensor detects that pilot is engaged. This sensor automatically switches the "engaging switch" on.
  At the time the pilot is engaged, the heading angle becomes the "target heading angle" 
  (Deviation heading angle) is :  (current heading angle) - (target heading angle)

  the program uses deviation heading and angular speed to compute whether the motor of the pilot must be run or not. It uses a PID (without "I"). 
  Motor must be run if the command computed by PID is outside the deadband. This deadbad can be adjusted by user by turning the "gain" potentiometer.
  If motor must be run, h-bridge connected to the board is activated, and current delivered to the motor is measured.
  If this current is too high (for example if the tiller is blocked by an obstacle), the signal to the h-bridge is paused.
