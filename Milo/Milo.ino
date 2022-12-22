/*
Source file of the Milo autopilot.
Philippe Dufossé 2022

Runs on a arduino-compatible nano board.
It may be edited using the arduino IDE (the arduino programming platform).
It is connected to a Bosch-sensortec BMX055 IMU (Inertial Measurement Unit)
It is also connected to a h-bridge for controling the DC motor of the autopilot.

It uses the FaBo9Axis_BMX055 library to communicate with the BMX055 IMU. This libray has been slightly modified in order to
maximize the accuracy and stability of the IMU, and renamed to "FaBo9Axis_BMX055_Milo"

The IMU is mounted on a gimbal. So all calculations are made directly in 2D, in the horizontal plane.
So this program uses only the x and y components of the IMU magnetometer, and only the z component of the IMU gyrometer; 
it does not use the accelerometer.


What this program do : 

- If the calibration switch is "on":

  User is requested to execute 2 circles with the boat, while doing this, the program computes the calibration offsets
   along x and y axis of the IMU. When this is done these offsets are stored in the EEPROM of the board, for further use.

- If the calibration switch is "off" :
  
  The program constantly computes the current heading angle and current angular speed of the boat.
  When user engages the pilot, a sensor detects that pilot is engaged. This sensor automatically switches the "engaging switch" on.
  At the time the pilot is engaged, the heading angle becomes the "target heading angle" 
  (Deviation heading angle) is :  (current heading angle) - (target heading angle)

  the program uses deviation heading and angular speed to compute whether the motor of the pilot must be run or not. It uses a PID (without "I"). 
  Motor must be run if the command computed by PID is outside the deadband. This deadbad can be adjusted by user by turning the "gain" potentiometer.
  If motor must be run, h-bridge connected to the board is activated, and current delivered to the motor is measured.
  If this current is too high (for example if the tiller is blocked by an obstacle), the signal to the h-bridge is paused.
*/
//---------------------------------------------------------------------------------------------

#include <Wire.h>
#include <FaBo9Axis_BMX055_Milo.h>
#include <EEPROM.h>

//  Pins connected to the h-bridge board
#define  IS_1  2      // pin connected to CW current mesurement
#define  IS_2  1      // pin connected to CCW current mesurement
#define  IN_1  6      // pin connected to the activation of CW sense of motor
#define  IN_2  7      // pin connected to the activation of CCW sense of motor
#define  IN_enable 12 // pin connected to all enabling entries of the h-bridge board.

// Other pins
#define  I_ENGAGE 11  // pin connected to the "engaging" switch
#define  I_CALIBR 4   // pin connected to the "calibration on/off" switch

// Some constants
#define  KHEAD   1.0  // For the PID : Coefficient of the "P" component of the PID
#define  KDERIV  3.0  // For the PID : Coefficient of the "D" component of the PID
#define  DEADBAND  7.0 //  Deadband. If command remains inside this deadband we do nothing.
#define  COEFF_GYRO_HEADING  0.5  // Coefficient used for the complementary filder applied to heading angle
#define  CURRENTMAX  1000         // Representing the maximum current of the h-bridge output. If measured current exceeds this value, we stop 

FaBo9Axis mySensor;      //Object that for the BMX055 IMU sensor 

// Some various variables, that must exist all along the program.
unsigned long CurrentTime=0.0;
int iRunPrevious = 0;
float TargetHeading, CurrentHeading;
int iEngage = 0;
int x_offset, y_offset;       //   calibration offsets, stored in EEPROM
int Current_Xmin,  Current_Xmax,  Current_Ymin, Current_Ymax;  // some variables used for calibration offsets calculation
int iCalibrPrevious = 1;

//---------------------------------------------------------------------------------------------
// SETUP FUNCTION - (This code is executed once)
//---------------------------------------------------------------------------------------------

void setup()
{    
  //Peripheral Initialization
  //Initialize the Serial Port to view information on the Serial Monitor
  Serial.begin(9600);  
  Wire.begin();          //Initialize I2C communication to the let the library communicate with the sensor.

  // IMU sensor initialization
  while(!mySensor.begin()){
    Serial.println("Not found 9Axis BMX055");
    delay(1000);
  }  
  // set up the pins connected to the h-bridge board
  pinMode(IN_1,OUTPUT);
  pinMode(IN_2,OUTPUT);
  digitalWrite(IN_1,0);
  digitalWrite(IN_2,0);
  

  pinMode(IN_enable,OUTPUT);
  digitalWrite(IN_enable, 1);

  // set up pin connected to "engaging switch"
  pinMode(I_ENGAGE,INPUT);
  
  // set up pin connected to "calibration" switch.
  // Note that by default, when we are not in calibration mode (switch is set to "off"), value of this digital pin is "1".
  pinMode(I_CALIBR,INPUT_PULLUP);
  
  CurrentTime = millis();

  // Read current magnetic offsets from EEPROM
  EEPROM.get(0, x_offset);
  EEPROM.get(2, y_offset);
 
  // calculate heading 10 times, for sensor stabilisation
  for (int index = 0; index < 10; index++) CurrentHeading = CalculateHeading();
  
}

//---------------------------------------------------------------------------------------------
// LOOP function - (This code is looped forever)
//---------------------------------------------------------------------------------------------

void loop() 
{
  int iRun = 0;
  float DeviationHeading, CalculatedHeading, Command, deadband, GyroRate, dt, deadband_correction;
  int icurrent1, icurrent2;
  int igyro[3];
  int iFirstTime;
  unsigned long NewCurrentTime;

   // CALIBRATION ---------------------------------------------------------
  
   // Test if we are in calibration mode. If yes (iCalibr = 0) : execute calibration (compute new offsets), and exit.
   // Note that by default, when we are not in calibration mode (switch is "off"), value of this digital pin is "1"
   // so in calibration mode, value of this pin is "0".

  int iCalibr = digitalRead(I_CALIBR);
  if (iCalibr == 0)
  {
    // We need to know if this is the first loop in "calibration mode"  (iFirstTime = 1)
    // This info will be used later by the ExecuteCalibration subroutine.   
    if (iCalibrPrevious == 1) iFirstTime = 1;
    else iFirstTime = 0;
    iCalibrPrevious = 0;
   
    ExecuteCalibration(iFirstTime);
     
    return;
  }

  // if in previous loop we were in calibration mode : store new offsets in EEPROM.
  if (iCalibrPrevious == 0) 
  {
    iCalibrPrevious = 1;
    StoreNewOffsetsInEEPROM();
  }
  
  // NOT IN CALIBRATION MODE: NORMAL HEADING COMPUTING AND MOTOR ACTIVATION -----------------------------

  
  // Calculate angular speed, around vertical axis (the z axis of the IMU), in degrees / sec.
  mySensor.readGyro(igyro);
  GyroRate = (double)igyro[2] * 0.0038;
    
  // Calculate current heading   (between -180 and 180 degrees).
  CalculatedHeading = CalculateHeading();
    
  // Solving the jump from -180 to 180 issue.
  if ( CalculatedHeading - CurrentHeading <= -180.0) CalculatedHeading += 360.0;
  else if ( CalculatedHeading - CurrentHeading >= 180.0) CalculatedHeading -= 360.0;
   
  // Apply a complementary filter to current heading
  // This is a combination of the calcultad heading, and of angular speed, integrated along the loop duration (dt).
  // This filter is used to remove some of the noise produced by the IMU magnetometer

  NewCurrentTime = millis();
  dt = float(NewCurrentTime - CurrentTime) / 1000.0;
  CurrentTime = NewCurrentTime;
  float CoeffHeading = 1.0 - COEFF_GYRO_HEADING;
  CurrentHeading = COEFF_GYRO_HEADING * (CurrentHeading + GyroRate * dt) + CoeffHeading * CalculatedHeading;
           
  // Solving the jump from -180 to 180 issue.
  if (CurrentHeading <= -180.0) CurrentHeading += 360.0;
  if (CurrentHeading > 180.0) CurrentHeading -= 360.0;

  if (digitalRead(I_ENGAGE) == 1)
  {
     // if the pilot has just been engaged, we capture the target heading.
     if (iEngage == 0)
     {
      iEngage = 1;
      TargetHeading = CurrentHeading; 
     }
  }
  else iEngage = 0; 
  
  if (iEngage == 1) 
  {
    // Calculate deviation of heading
    DeviationHeading = CurrentHeading - TargetHeading; 
  
    // Solving (again...) the jump from -180 to 180 issue.
    if (DeviationHeading <= -180.0) DeviationHeading += 360.0;
    if (DeviationHeading > 180.0) DeviationHeading -= 360.0; 
 
    // Calculating the "command" that will be used later to decide whether the pilot must run or not.
    // Here we use a "PID" without "I", combining the heading deviation and the angular speed.
    Command = KHEAD * DeviationHeading + KDERIV * GyroRate;
      
    // If command is outside of deadband : motor of pilot must run.
    // Actual deadband depends on position of the "gain" potentiometer defining the value of "deadband_correction".
    // this potentiometer is connected to the A0 pin.
    // Note: the potentiometer is presented as "gain" potentiometer to the user, but in fact it affects the deadband, not the gain of the pilot.
    
    int val  = analogRead(A0);  // read the A0 input pin
    deadband_correction = ((float) val * 12.0 / 1023.0) - 6.0;  // correction of deadband : -6 to 6
    deadband = DEADBAND + deadband_correction;
        
    // Here we define whether motor of pilot must run clockwise or counter-clockwise.
    if (Command >= deadband) iRun = 1;
    else if (Command <= -deadband) iRun = -1; 
   }

  /*
  Serial.print("iEngage:  ");
  Serial.print(iEngage);
  Serial.print("  /  TargetHead: ");
  Serial.print(TargetHeading);
  Serial.print("  /  CurrentHead: ");
  Serial.print(CurrentHeading);
  Serial.print("  /  DeviationHead: ");
  Serial.print(DeviationHeading);
  Serial.print("  /  GyroRate: ");
  Serial.print(GyroRate);
  Serial.print("  /  deadband_corr: ");
  Serial.print(deadband_correction);        
  Serial.print("  /  deadband: ");
  Serial.print(deadband);       
  Serial.print("  /  Command: ");
  Serial.print(Command);
  Serial.print("  /  Run: ");
  Serial.println(iRun);
  */

  // inside deadband : we do nothing
  if (iRun == 0)
  {
     digitalWrite(IN_1, 0) ;
     digitalWrite(IN_2, 0) ;
  }
  // above deaband : motor must run clockwise
  if (iRun == 1)
  {
     digitalWrite(IN_2, 0) ;
     digitalWrite(IN_1, 1) ;
  }
  // below deaband : motor must run counter-clockwise
  if (iRun == -1)
  {
     digitalWrite(IN_1, 0) ;
     digitalWrite(IN_2, 1) ;
  }
 

  // Measure current on both branches of the h-bridge board. If too high pause for a few seconds
  // But we do it only from the second loop of the same "iRun", because in the first loop the measured current is always very high anyway.
  
  if (iRun != iRunPrevious) iRunPrevious = iRun;
    
  else 
  {

    if (iRun != 0)
     {
     icurrent1 = 0 , icurrent2 = 0;
     icurrent1 = analogRead(IS_1);    
     if (icurrent1 < CURRENTMAX) icurrent2 = analogRead(IS_2); 
    
     if (icurrent1 >= CURRENTMAX  || icurrent2 >= CURRENTMAX)
     {
       digitalWrite(IN_1, 0) ;
       digitalWrite(IN_2, 0) ;        
       Serial.print(" current 1 :  ");
       Serial.print(icurrent1);
       Serial.print(" current 2 :  ");
       Serial.println(icurrent2);
       delay(5000);  
      }
    }
  }
}

//---------------------------------------------------------------------------------------------
//    SUBROUTINES
//---------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------
//  Heading calculation
//---------------------------------------------------------------------------------------------

float CalculateHeading()
{
  float  x, y, angle;
  int imag[3];

  // Read sensor values and deduct calibration offsets
  mySensor.readMag(imag) ;    
  x = (float)(imag[0] - x_offset);
  y = (float)(imag[1] - y_offset);
  
  angle = atan2(x, y) * RAD_TO_DEG; 
  return angle;
}


//---------------------------------------------------------------------------------------------
//  Executing the calibration, when we are in "calibation mode".
//  Here we compute min and max values of x and y values read from IMU, as the boat is executing the two circles.
//  Later they will be used to calculate calibration offsets
//---------------------------------------------------------------------------------------------

void ExecuteCalibration(int iFirstTime)
{
  int VMag[3];
  
  // Read Magnetic vector -> X:VMag[0], Y:VMag[1]
  mySensor.readMag(VMag);
  
  // If we are in the first loop of calibation mode, just initialize min and max from read values
  if (iFirstTime == 1) 
  {
  Current_Xmin = VMag[0];
  Current_Xmax = VMag[0];
  Current_Ymin = VMag[1];
  Current_Ymax = VMag[1];
  }
  
  else
  {
  if (VMag[0] > Current_Xmax) Current_Xmax = VMag[0];
  if (VMag[1] > Current_Ymax) Current_Ymax = VMag[1];
  if (VMag[0] < Current_Xmin) Current_Xmin = VMag[0];
  if (VMag[1] < Current_Ymin) Current_Ymin = VMag[1];
  }
}


//---------------------------------------------------------------------------------------------
//   Calculate new calibration offsets and store them in EEPROM
//---------------------------------------------------------------------------------------------

void StoreNewOffsetsInEEPROM()
{
  // Calculate new offsets from min and max values of x and y. 
  int xoffsetNew = (Current_Xmax + Current_Xmin) / 2.0;
  int yoffsetNew = (Current_Ymax + Current_Ymin) / 2.0;

  // if new offsets are almost same as stored offsets : do nothing
  if (fabs (x_offset - xoffsetNew) > 1.0 || fabs (y_offset - yoffsetNew) > 1.0) 
  {
    x_offset = xoffsetNew;
    y_offset = yoffsetNew;
    
    // store in eeprom. x_offset is stored in bytes 0 and 1 of EEPROM; y_offset is stored in bytes 2 and 3.
    EEPROM.put(0, x_offset);
    EEPROM.put(2, y_offset);
  }
}