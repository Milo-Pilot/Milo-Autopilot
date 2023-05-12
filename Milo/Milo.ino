/*
Source file of the Milo autopilot.


Runs on a arduino-compatible nano board.
It may be edited using the arduino IDE (the arduino programming platform).
It is connected to a Bosch-sensortec BMX055 IMU (Inertial Measurement Unit)
It is also connected to a h-bridge for controling the DC motor of the autopilot.

It uses the FaBo9Axis_BMX055 library to communicate with the BMX055 IMU. This libray has been slightly modified in order to
maximize the accuracy and stability of the IMU, and renamed to "FaBo9Axis_BMX055_Milo"

What this program do : 

- If the calibration switch is "on":

  User is requested to execute 2 circles with the boat, while doing this, the program progressively computes some variables
  that will be used later to compute calibration offsets. These offsets will be stored in the EEPROM of the nano board.
  
  
- If the calibration switch is "off" :
  
  The program constantly computes the current heading angle and current angular speed of the boat.
  When user engages the pilot, a sensor detects that pilot is engaged. This sensor automatically switches the "engaging switch" on.
  At the time the pilot is engaged, the heading angle becomes the "target heading angle" 
  (Deviation heading angle) is :  (current heading angle) - (target heading angle)

  the program uses deviation heading and angular speed to compute whether the motor of the pilot must be run or not. It uses a PID (without "I"). 
  Motor must be run if the command computed by PID is outside the deadband. 
  If motor must be run, h-bridge connected to the board is activated, and current delivered to the motor is measured.
  If this current is too high (for example if the tiller is blocked by an obstacle), the signal to the h-bridge is paused.
*/
//---------------------------------------------------------------------------------------------

#include <Wire.h>
#include <FaBo9Axis_BMX055_Milo.h>
#include <EEPROM.h>

//  Pins connected to the h-bridge board
#define  IS_1  0      // 2  pin connected to CW current mesurement
#define  IS_2  1      // 1  pin connected to CCW current mesurement
#define  IN_1  10     // 1 6  pin connected to the activation of CW sense of motor
#define  IN_2  11     // 16 7  pin connected to the activation of CCW sense of motor
#define  IN_enable 12 // 17 12  pin connected to all enabling entries of the h-bridge board.

// Other pins
#define  I_ENGAGE 16  // 11 pin connected to the "engaging" switch
#define  I_CALIBR 17  // 4 pin connected to the "calibration on/off" switch

// Some constants
#define  VAL_ZERO 0.01
#define  COEFF_GYRO  0.95 // 0.9 // 0.7           //
#define  KHEAD   1.0              // For the PID : Coefficient of the "P" component of the PID
#define  KDERIV  4.0              // For the PID : Coefficient of the "D" component of the PID
#define  DEADBAND  7.0            //  Deadband. If command remains inside this deadband we do nothing.
#define  COEFF_GYRO_HEADING  0.5  // Coefficient used for the complementary filder applied to heading angle
#define  CURRENTMAX  250          // Representing the maximum current of the h-bridge output. If measured current exceeds this value, we stop 
#define  MAXHEALING  0.95         // Used for the calibration. If scalar product of the current healing angle is below this value,
                                  // that means that the boat is not flat; this loop is skipped.

//   EEPROM addresses of data stored in EEPROM :
//   Calibration status is an int, so it occupies 2 bytes.  
//   All other values are float, so each one occupies 4 bytes.

#define  ADD_calib_status  0  // calibration status (int 0 : not calibrated 1 : partially calibrated)  2 : fully calibrated
#define  ADD_x_Off  2       // x, y z, final offsets
#define  ADD_y_Off  6
#define  ADD_z_Off  10

FaBo9Axis mySensor;      //Object that for the BMX055 IMU sensor 

// Some various variables, that must exist all along the program.
unsigned long CurrentTimeLoop=0.0;
unsigned long CurrentTimeGrav=0.0;
int iRunPrevious = 0;
int iRunTooHighCurrent = 0;              // rotation sense with too high current
float TargetHeading, CurrentHeading;
int iEngage = 0;
int Current_Xmin,  Current_Xmax,  Current_Ymin, Current_Ymax;  // some variables used for calibration offsets calculation
int iCalibrPrevious = 1;
float GyroRate = 0.0;
int calibration_status;
float x_Offset, y_Offset, z_Offset;     //   calibration offsets, stored in EEPROM

float VGyro[3]={0.0, 0.0, 0.0};         // Gyrometer vector read from the IMU
float VMag[3]={0.0, 0.0, 0.0};          // Earth magnetic vector read from the IMU
float VAccel[3]={0.0, 0.0, 0.0};        // Acceleration vector read from the IMU
float VGrav[3]={0.0, 0.0, 0.0};         // Gravity vector
float VVert[3]={0.0, 0.0, 0.0};         // Vertical vector
float VRefBoat[3]={0.0, 0.0, 0.0};      // Reference vector of the boat
float AngleGrav[3]={0.0, 0.0, 0.0};     // angles of the gravity vector
float VXRefFrame[3]={0.0, 0.0, 0.0};    // X axis of the horizontal reference frame
float VYRefFrame[3]={0.0, 0.0, 0.0};    // Y axis of the horizontal reference frame
float VZRefFrame[3]={0.0, 0.0, 0.0};    // Z axis of the horizontal reference frame
float OldCalculatedHeading = 0.0;       // for thr low-pass filter appied to the heading

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
    Serial.println("BMX055 IMU not found");
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
  
  CurrentTimeLoop = millis();
  CurrentTimeGrav = CurrentTimeLoop;

  // Read current calibration data from EEPROM (if calibration is done)
  EEPROM.get(ADD_calib_status, calibration_status);

  if (calibration_status == 1)
  {
    EEPROM.get(ADD_x_Off, x_Offset);
    EEPROM.get(ADD_y_Off, y_Offset);
    EEPROM.get(ADD_z_Off, z_Offset);
    /* 
    Serial.println("DEBUT COMPLETED CALIBRATION");
    Serial.print("x_Offset:  ");
    Serial.println(x_Offset);
    Serial.print("y_Offset:  ");
    Serial.println(y_Offset);
    Serial.print("z_Offset:  ");
    Serial.println(z_Offset);
    */
  }
 
  // Read data from the IMU
  ReadIMU();
  
  // Calculate initial gravity vector.
  CalculateGravity();

  // Choose the reference vector attached to the boat  _ Should be the most possible orthogonal to the gravity vector.
  float psx = fabs(VGrav[0]);
  float psy = fabs(VGrav[1]);
  float psz = fabs(VGrav[2]);  
  if (psx <= psy && psx <= psz) VRefBoat[0] = 1.0;
  else
  {
    if (psy <= psx && psy <= psz)VRefBoat[1] = 1.0;
    else VRefBoat[2] = 1.0;  
  }

 // Calculate horizontal reference frame and initial heading (to initialize the CurrentHeading variable)
  CalculateHorReferenceFrame(VGrav, VXRefFrame, VYRefFrame, VZRefFrame);
  CurrentHeading = CalculateHeading(); 
  OldCalculatedHeading = CurrentHeading;

}

//---------------------------------------------------------------------------------------------
// LOOP function - (This code is looped forever)
//---------------------------------------------------------------------------------------------

void loop() 
{
  int iRun = 0;
  float DeviationHeading, CalculatedHeading, Command, dt;
  int icurrent1, icurrent2;
  int iFirstTime;
  unsigned long NewCurrentTime;

  // Read data from the IMU, calculate gravity vector and calculate horizontal reference frame
  ReadIMU();
  CalculateGravity();
  CalculateHorReferenceFrame(VGrav, VXRefFrame, VYRefFrame, VZRefFrame);
  
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
    iEngage = 0;
  }
  
  //-----------------------------------------------------------------------------------------------------
  // NOT IN CALIBRATION MODE: NORMAL HEADING COMPUTING AND MOTOR ACTIVATION -----------------------------
  //-----------------------------------------------------------------------------------------------------

  // Calculate current heading   (between -180 and 180 degrees).
  CalculatedHeading = CalculateHeading();
  
   // Solving the jump from -180 to 180 issue.
  if ( CalculatedHeading - CurrentHeading <= -180.0) CalculatedHeading += 360.0;
  else if ( CalculatedHeading - CurrentHeading >= 180.0) CalculatedHeading -= 360.0;
 

  // low pass filter applied to heading
  float NewCalculatedHeading = 0.5 * CalculatedHeading + 0.5 * OldCalculatedHeading;
  CalculatedHeading = NewCalculatedHeading;
  OldCalculatedHeading = CalculatedHeading;

  // Apply a complementary filter to current heading
  // This is a combination of the calculated heading, and of angular speed, integrated along the loop duration (dt).
  // This filter is used to remove some of the noise produced by the IMU magnetometer

  NewCurrentTime = millis();
  dt = float(NewCurrentTime - CurrentTimeLoop) / 1000.0;
  CurrentTimeLoop = NewCurrentTime;
  float CoeffHeading = 1.0 - COEFF_GYRO_HEADING;
  
  CurrentHeading = COEFF_GYRO_HEADING * (CurrentHeading + GyroRate * dt) + CoeffHeading * CalculatedHeading;
    
  // Solving the jump from -180 to 180 issue.
  if (CurrentHeading <= -180.0) CurrentHeading += 360.0;
  if (CurrentHeading > 180.0) CurrentHeading -= 360.0;

  
  //if (1)
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
    // Here we define whether motor of pilot must run clockwise or counter-clockwise.
    if (Command >= DEADBAND) iRun = 1;
    else if (Command <= -DEADBAND) iRun = -1; 
   }

/*
  Serial.print("TargHd:");
  Serial.print(TargetHeading);
  Serial.print(",");
  Serial.print("CurHd:");
  Serial.print(CurrentHeading);
  Serial.print(",");  
  Serial.print("DevHd:");
  Serial.print(DeviationHeading);
 //Serial.println();

  Serial.print("  /  GyrRat: ");
  Serial.print(GyroRate);
 
  
  Serial.print("  /  deadb_corr: ");
  Serial.print(deadband_correction);
  Serial.print("  /  kderiv_corr: ");
  Serial.print(kderiv_correction);
         
  Serial.print("  /  deadb: ");
  Serial.print(deadband);       
  Serial.print("  /  Commd: ");
  Serial.print(Command);
  Serial.print("  /  Run: ");
  Serial.println(iRun);
*/
  // If previously measured current was too high and motor wants to turn in same sense --> stop.
  if (iRun == iRunTooHighCurrent) iRun = 0;
  else iRunTooHighCurrent = 0;

    // inside deadband : we do nothing
  if (iRun == 0)
  {
     digitalWrite(IN_1, 0) ;
     digitalWrite(IN_2, 0) ;
  }
  // above deaband : motor must run clockwise
  if (iRun == -1)
  {
     digitalWrite(IN_2, 0) ;
     digitalWrite(IN_1, 1) ;
  }
  // below deaband : motor must run counter-clockwise
  if (iRun == 1)
  {
     digitalWrite(IN_1, 0) ;
     digitalWrite(IN_2, 1) ;
  }
 
  // Measure current on both branches of the h-bridge board. If too high pause for a few seconds
  // But we do it only from the second loop of the same "iRun", because in the first loop the measured current is always very high anyway.
  // Motor will restart if it turns in opposite sense.
  
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
       iRunTooHighCurrent = iRun;
       digitalWrite(IN_1, 0) ;
       digitalWrite(IN_2, 0) ;        
       Serial.print("CURRENTMAX   current 1 :  ");
       Serial.print(icurrent1);
       Serial.print("   current 2 :  ");
       Serial.println(icurrent2);
       delay(3000);  
      }


/*
      Serial.print("CURRENT   current 1 :  ");
      Serial.print(icurrent1);
      Serial.print("   current 2 :  ");
      Serial.println(icurrent2);
 */     
   
    }
  }
}

//---------------------------------------------------------------------------------------------
//    SUBROUTINES
//---------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------
//  Read Gyro vector (VGyro), magnetic vector (VMag), acceleration vector (VAccel) from the IMU.
//---------------------------------------------------------------------------------------------

void ReadIMU()
{
  int vect[3], index;
  mySensor.readGyro(vect);
  // in degrees / second
  for (index = 0; index < 3; index++) VGyro[index] = (float) vect[index] * 0.0038;

  // Refer to the data sheet of the BMX055: x axis of the magnetometer corresponds to the y axis of the acceleromet and gyro, and it is inverted.
  mySensor.readMag(vect) ;
  // for (index = 0; index < 3; index++) VMag[index] = (float) vect[index];   
  VMag[0] = (float) ( -vect[1]); 
  VMag[1] = (float) vect[0];
  VMag[2] = (float) vect[2]; 

  mySensor.readAccel(vect) ;    
  for (index = 0; index < 3; index++) VAccel[index] = (float) vect[index];
}

//---------------------------------------------------------------------------------------------
//  Heading calculation
//---------------------------------------------------------------------------------------------

float CalculateHeading()
{
  // if calibration is done : deduct offsets from mag vector
  if (calibration_status == 1)
  {
    VMag[0] -= x_Offset;
    VMag[1] -= y_Offset;
    VMag[2] -= z_Offset;
  }  

  // Compute magnetic vector in horizontal reference frame
  float x = VScalarProduct(VXRefFrame, VMag);
  float y = VScalarProduct(VYRefFrame, VMag);

  return atan2(x, y) * RAD_TO_DEG; 
}

//---------------------------------------------------------------------------------------------
//  Executing the calibration, when we are in "calibration mode".
//  Here we compute min and max values of x and y values read from IMU, in a "horizontal" reference frame", as the boat is executing the two circles.
//  Later they will be used to calculate calibration offsets
//---------------------------------------------------------------------------------------------

void ExecuteCalibration(int iFirstTime)
{
  float xflat, yflat, zflat;
  int index, index1;

  // Calculate vertical vector
  // At this time, the boat is supposed to be "flat", no healing.
  // VVert (vertical vector) is the mean of 10 times calculated gravity vectors.
  if (iFirstTime == 1)
  {
    for (index = 0; index < 3; index++) VVert[index] = 0.0;
    for (index1 = 0; index1 < 10; index1++)
    {
      CalculateGravity();
      for (index = 0; index < 3; index++) VVert[index] += VGrav[index];
    }
    for (index = 0; index < 3; index++) VVert[index] /= 10.0;
    VNormalize(VVert); 
  }
  
  // If the boat is healing too much : exit
  if (iFirstTime == 0 && VScalarProduct (VGrav, VVert) < MAXHEALING) return;
  
  xflat = VScalarProduct (VXRefFrame, VMag);
  yflat = VScalarProduct (VYRefFrame, VMag);
 
  // If we are in the first loop of calibration mode, just initialize min and max from read values
  if (iFirstTime == 1) 
  {
    Current_Xmin = xflat;
    Current_Xmax = xflat;
    Current_Ymin = yflat;
    Current_Ymax = yflat;
  }
  
  else
  {
    if (xflat > Current_Xmax) Current_Xmax = xflat;
    if (yflat > Current_Ymax) Current_Ymax = yflat;
    if (xflat < Current_Xmin) Current_Xmin = xflat;
    if (yflat < Current_Ymin) Current_Ymin = yflat;
  }
}

//---------------------------------------------------------------------------------------------
//   Calculate new calibration offsets and store them in EEPROM
//---------------------------------------------------------------------------------------------

void StoreNewOffsetsInEEPROM()
{
  float x_FlatOffset, y_FlatOffset;           //  offsets in horizontal plane
  float VX_VertRefFrame[3];
  float VY_VertRefFrame[3];
  float VZ_VertRefFrame[3];
   
  // New calibration status : 1
  if (calibration_status != 1) 
  {
    calibration_status = 1;
    EEPROM.put(ADD_calib_status, calibration_status);
  }
  
  // Calculate new offsets from min and max values of x and y. 
  // and store resulting values in EEPROM

  x_FlatOffset = (Current_Xmax + Current_Xmin) / 2.0;
  y_FlatOffset = (Current_Ymax + Current_Ymin) / 2.0;
  
  // Calculate offsets in local reference frame
  CalculateHorReferenceFrame(VVert, VX_VertRefFrame, VY_VertRefFrame, VZ_VertRefFrame);
  x_Offset =  VX_VertRefFrame[0] * x_FlatOffset + VY_VertRefFrame[0] * y_FlatOffset;
  y_Offset =  VX_VertRefFrame[1] * x_FlatOffset + VY_VertRefFrame[1] * y_FlatOffset;
  z_Offset =  VX_VertRefFrame[2] * x_FlatOffset + VY_VertRefFrame[2] * y_FlatOffset;

  // Store final offsets in EEPROM.
  EEPROM.put(ADD_x_Off, x_Offset);
  EEPROM.put(ADD_y_Off, y_Offset);
  EEPROM.put(ADD_z_Off, z_Offset);
}


//---------------------------------------------------------------------------------------------
// Calculate horizontal reference frame from vertical vector
//---------------------------------------------------------------------------------------------

void CalculateHorReferenceFrame(float VertVector[], float VX[], float VY[], float VZ[] )
{
  VZ[0] = VertVector[0]; 
  VZ[1] = VertVector[1]; 
  VZ[2] = VertVector[2]; 
  VectorialProduct(VRefBoat, VZ, VX);
  VNormalize(VX);
  VectorialProduct(VZ, VX, VY);
}

//---------------------------------------------------------------------------------------------
// Calculate Gravity vector
//---------------------------------------------------------------------------------------------

void CalculateGravity()
{
  float AngleAccel[3], dt;
  unsigned long NewCurrentTime;

  VNormalize(VAccel);
 
  // First calculation of VGrav : Vgrav and its angles directly taken from acceleration vector
  // Other following calculations : complementary filter is used.
  
  if (VGrav[0] == 0.0 && VGrav[1] == 0.0 && VGrav[2] == 0.0)
  {
    CurrentTimeGrav = millis(); 
    VGrav[0] = VAccel[0];
    VGrav[1] = VAccel[1];
    VGrav[2] = VAccel[2];
    AngleFromVector(VAccel, AngleGrav);
  }
  else 
  {  
    AngleFromVector(VAccel, AngleAccel);
  
    NewCurrentTime = millis(); 
    dt = float(NewCurrentTime - CurrentTimeGrav) / 1000.0;
    CurrentTimeGrav = NewCurrentTime;

    // Calculate the gravity angles using a complementary filter
    // This filter combines acceleration data and gyro data. Here data is seen as the derivative of the angles of the acceleration vector.     
    if ( AngleAccel[0] - AngleGrav[0] <= -180.0) AngleAccel[0] += 360.0;
    else if ( AngleAccel[0] - AngleGrav[0] >= 180.0) AngleAccel[0] -= 360.0;
    if ( AngleAccel[1] - AngleGrav[1] <= -180.0) AngleAccel[1] += 360.0;
    else if ( AngleAccel[1] - AngleGrav[1] >= 180.0) AngleAccel[1] -= 360.0;
    if ( AngleAccel[2] - AngleGrav[2] <= -180.0) AngleAccel[2] += 360.0;
    else if ( AngleAccel[2] - AngleGrav[2] >= 180.0) AngleAccel[2] -= 360.0;

    float coeffAngle = 1.0 - COEFF_GYRO;
    AngleGrav[0] = COEFF_GYRO * (AngleGrav[0] + VGyro[0] * dt) + coeffAngle * AngleAccel[0]; 
    AngleGrav[1] = COEFF_GYRO * (AngleGrav[1] + VGyro[1] * dt) + coeffAngle * AngleAccel[1]; 
    AngleGrav[2] = COEFF_GYRO * (AngleGrav[2] + VGyro[2] * dt) + coeffAngle * AngleAccel[2]; 
 
    VectorFromAngle(VGrav, AngleGrav);  
   }
   
  VNormalize(VGrav);
/*
  Serial.print("GravX:");
  Serial.print(VGrav[0]);
  Serial.print(",");
  Serial.print("GravY:");
  Serial.print(VGrav[1]);
  Serial.print(",");
  Serial.print("GravZ:");
  Serial.println(VGrav[2]);
*/

  // Calculate the rotation factor that will be used as the derivative part of the PID .
  // This is the rotation speed around the gravity vector. 
  // rotation around each x,y,z axis contributes with a factor of scalar product (axis, gravity vector).
  // in degrees per second
  GyroRate = VGyro[0] * VGrav[0] +  VGyro[1] * VGrav[1] +  VGyro[2] * VGrav[2];
}


//---------------------------------------------------------------------------------------------
// OTHER LOCAL UTILITIES 
//---------------------------------------------------------------------------------------------

float VNorm(float Vector[])
{
  return sqrt (Vector[0]*Vector[0] + Vector[1]*Vector[1] + Vector[2]*Vector[2]);
}

void VNormalize(float Vector[])
{
  float Norm = VNorm(Vector);

  if (Norm  > 0.01)
  {
    Vector[0] = Vector[0] / Norm;
    Vector[1] = Vector[1] / Norm;
    Vector[2] = Vector[2] / Norm;
  }
}

void VectorialProduct(float V1[], float V2[], float VResult[])
{
  VResult[0] = (V1[1] * V2[2]) - (V1[2] * V2[1]);
  VResult[1] = (V1[2] * V2[0]) - (V1[0] * V2[2]);
  VResult[2] = (V1[0] * V2[1]) - (V1[1] * V2[0]);
}

float VScalarProduct(float V1[], float V2[])
{
    return (V1[0]*V2[0] + V1[1]*V2[1] + V1[2]*V2[2]);
}

void AngleFromVector(float Vector[], float Angle[])
{
  // Calculate angles in degrees, from -180 to + 180.
  Angle[0] = atan2(Vector[2], Vector[1]) * RAD_TO_DEG;
  Angle[1] = atan2(Vector[0], Vector[2]) * RAD_TO_DEG;
  Angle[2] = atan2(Vector[1], Vector[0]) * RAD_TO_DEG;
}

void VectorFromAngle(float Vector[], float Angle[])
{
  float cosax, sinax, cosay, sinay, cosaz, sinaz, x, y, z;
 
  cosax = cos (Angle[0] * DEG_TO_RAD);
  sinax = sin (Angle[0] * DEG_TO_RAD);
  cosay = cos (Angle[1] * DEG_TO_RAD);
  sinay = sin (Angle[1] * DEG_TO_RAD);
  cosaz = cos (Angle[2] * DEG_TO_RAD);
  sinaz = sin (Angle[2] * DEG_TO_RAD);
  
  // Calculating x
  if (fabs(cosaz) < VAL_ZERO  || fabs(sinay) < VAL_ZERO) x = 0.0;
  else
    {
      x = 1.0 / sqrt(1.0 + sq(sinaz / cosaz) + sq(cosay / sinay));
      if (cosaz < 0.0) x = -x;
    }
  
   // Calculating y
  if (fabs(cosax) < VAL_ZERO  || fabs(sinaz) < VAL_ZERO) y = 0.0;
  else
    {
      y = 1.0 / sqrt(1.0 + sq(sinax / cosax) + sq(cosaz / sinaz));
      if (cosax < 0.0) y = -y;
    }
  
   // Calculating z
  if (fabs(cosay) < VAL_ZERO  || fabs(sinax) < VAL_ZERO) z = 0.0;
  else
    {
      z = 1.0 / sqrt(1.0 + sq(sinay / cosay) + sq(cosax / sinax));
      if (cosay < 0.0) z = -z;
    }

  Vector[0] = x;
  Vector[1] = y;
  Vector[2] = z;
}