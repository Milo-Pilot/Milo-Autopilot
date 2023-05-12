/**
 @file read9axis.FaBo9Axis_BMX055.cpp
 @brief This is an Example for the FaBo 9Axis(BMX055) I2C Brick.

   http://fabo.io/219.html

   Released under APACHE LICENSE, VERSION 2.0

   http://www.apache.org/licenses/

 @author FaBo<info@fabo.io>
*/

/* This file has been modified for the Milo autopilot
   to improve accuracy and stability of the magnetometer (x and y axis).
*/ 

#include "FaBo9Axis_BMX055_Milo.h"

#include "Arduino.h"
#include "Wire.h"

int accl_addr;
int gyro_addr;
int mag_addr;

/**
 @brief Constructor
 * @param [in] accl Accel Device Address
 * @param [in] gyro Gyro Device Address
 * @param [in] mag  Mag Device Address
*/
FaBo9Axis::FaBo9Axis(int accl, int gyro, int mag){
  // set device address
  accl_addr = accl;
  gyro_addr = gyro;
  mag_addr  = mag;

  Wire.begin();
}

/**
 @brief Begin Device
 @retval true normaly done
 @retval false device error
*/
bool FaBo9Axis::begin() {
  if ( searchDevice() ) {
    configuration();
    return true;
  } else {
    return false;
  }
}

/**
 * @brief Serch bmx055
 * @retval true  Found Device
 * @retval false : Not Found Device
 */
bool FaBo9Axis::searchDevice()
{
  byte device = 0x00;
  readI2c(accl_addr, BMX055_WHO_AM_I_REG, 1, &device);

  if(device == BMX055_ACC_DEVICE){
    return true;
  } else{
    return false;
  }
}

/**
 * @brief Set Config
 */
void FaBo9Axis::configuration()
{
  /* SoftReset */
  // Accel SoftReset
  writeI2c(accl_addr, BMX055_RESET_REG, BMX055_INITIATED_SOFT_RESET);
  delay(2);  // wait 1.8ms
  // Gyro SoftReset
  writeI2c(gyro_addr, BMX055_RESET_REG, BMX055_INITIATED_SOFT_RESET);
  delay(2);  // wait 1.8ms
  // Mag SoftReset
  writeI2c(mag_addr, BMX055_MAG_POW_CTL_REG, BMX055_MAG_POW_CTL_SOFT_RESET);
  delay(2);

  /* Accel Setting */
  // Select Accel PMU Range(+-2)
  writeI2c(accl_addr, BMX055_ACC_PMU_RANGE_REG, BMX055_ACC_RANGE_2);
  // Select Accel PMU_BW   (7.81Hz)
  writeI2c(accl_addr, BMX055_ACC_PMU_BW_REG, BMX055_ACC_PMU_BW_7_81);
  // Select Accel PMU_LPW  (NomalMode, SleepDuration 0.5ms)
  writeI2c(accl_addr, BMX055_ACC_PMU_LPW_REG, BMX055_ACC_PMU_LPW_MODE_NOMAL|BMX055_ACC_PMU_LPW_SLEEP_DUR_0_5MS);

  /* Gyro Setting */
  // Select Gyro Range(262.4 LSB/°/s)
  writeI2c(gyro_addr, BMX055_GYRO_RANGE_REG, BMX055_GYRO_RANGE_262_4);
  // Select Gyro BW   (32Hz)
  writeI2c(gyro_addr, BMX055_GYRO_BW_REG, BMX055_GYRO_BW_32);
  // Select Gyro LPM1 (NomalMode, SleepDuration 2ms)
  writeI2c(gyro_addr, BMX055_GYRO_LPM1_REG,
    BMX055_GYRO_LPM1_MODE_NOMAL|BMX055_GYRO_LPM1_SLEEP_DUR_2MS);

  /* Mag Setting */
  // set sleep mode
  writeI2c(mag_addr, BMX055_MAG_POW_CTL_REG, BMX055_MAG_POW_CTL_SLEEP_MODE);
  delay(100);
  // adv.st, DataRate, OperationMode, SelfTest (NomalMode, ODR 10Hz)
  // writeI2c(mag_addr, BMX055_MAG_ADV_OP_OUTPUT_REG, BMX055_MAG_DATA_RATE_10);
  // Repetitions for X-Y Axis  0x04 -> 0b00000100 -> (1+2(2^2)) = 9
  // writeI2c(mag_addr, BMX055_MAG_REP_XY_REG, 0x04);
  // Repetitions for Z-Axis  0x0F-> 0b00001111-> (1 +(2^0 + 2^1 + 2^2 + 2^3) = 15
  // writeI2c(mag_addr, BMX055_MAG_REP_Z_REG, 0x0F);

  // Modifications for Milo --------------------------

  // adv.st, DataRate, OperationMode, SelfTest (NomalMode, ODR 20Hz)
  writeI2c(mag_addr, BMX055_MAG_ADV_OP_OUTPUT_REG, BMX055_MAG_DATA_RATE_20);
  // Set Normal mode
  //  writeI2c(mag_addr, BMX055_MAG_ADV_OP_OUTPUT_REG, BMX055_MAG_OP_MODE_NORMAL);
  // Repetitions for X-Y Axis  0x17 = 47
  // Repetitions for X-Y Axis  0x2F = 47
  writeI2c(mag_addr, BMX055_MAG_REP_XY_REG, 0x2F);
  // Repetitions for Z-Axis  0x01 = 1
  // Repetitions for Z-Axis  0x53 = 83
  writeI2c(mag_addr, BMX055_MAG_REP_Z_REG, 0x53);

  // End of modifications for Milo --------------------------
  
delay(200);
}

/**
 * @brief Read Accel
 * @param [out] *accl : accel value  (X-accel : accl[0], Y-accel : accl[1], Z-accel : accl[2])
 */
 void FaBo9Axis::readAccel(int *accl)
{
  uint8_t accl_data[6];

  // read accel value
  for (int i = 0; i < 6; i++)
  {
    readI2c(accl_addr, BMX055_ACC_DATA_START_REG+i, 1, &accl_data[i]);
  }

  // conv data  accel:12bit
  accl[0] = ((accl_data[1]<<4) + (accl_data[0]>>4));

  if (accl[0] > 2047)
  {
    accl[0] -= 4096;
  }

  accl[1] = ((accl_data[3]<<4) + (accl_data[2]>>4));
  if (accl[1]> 2047)
  {
    accl[1] -= 4096;
  }

  accl[2] = ((accl_data[5]<<4) + (accl_data[4]>>4));
  if (accl[2] > 2047)
  {
    accl[2] -= 4096;
  }
}

/**
 * @brief Read Gyro
 * @param [out] *gyro gyro value (X-gyro: gyro[0], Y-gyro: gyro[1], Z-gyro: gyro[2])
 */
void FaBo9Axis::readGyro(int *gyro)
{
  uint8_t gyro_data[6];

  // read gyro value
  for (int i = 0; i < 6; i++)
  {
    readI2c(gyro_addr, BMX055_GYRO_DATA_START_REG+i, 1, &gyro_data[i]);
  }

  // conv data  gyro:16bit
  gyro[0] = ((gyro_data[1]<<8) + gyro_data[0]);
  if (gyro[0] > 32767)
  {
    gyro[0] -= 65536;
  }

  gyro[1] = ((gyro_data[3]<<8) + gyro_data[2]);
  if (gyro[1] > 32767)
  {
    gyro[1] -= 65536;
  }

  gyro[2] = ((gyro_data[5]<<8) + gyro_data[4]);
  if (gyro[2] > 32767)
  {
    gyro[2] -= 65536;
  }
}

/**
 * @brief Read Mag
 * @param [out] *mag mag value (X-mag: mag[0], Y-mag: mag[1], Z-mag: mag[2])
 */
void FaBo9Axis::readMag(int *mag)
{
  uint8_t mag_data[6];

  // read mag value
  for (int i = 0; i < 6; i++)
  {
    readI2c(mag_addr, BMX055_MAG_DATA_START_REG+i, 1, &mag_data[i]);
  }

  // conv data  mag x:12bit
  mag[0] = ((mag_data[1]<<5) + (mag_data[0]>>3));
  if (mag[0] > 4095)
  {
    mag[0] -= 8192;
  }

  // conv data  mag y:12bit
  mag[1] = ((mag_data[3]<<5) + (mag_data[2]>>3));
  if (mag[1] > 4095)
  {
    mag[1] -= 8192;
  }

// Modifications for Milo --------------------------

  // conv data  mag z:15bit
  mag[2] = ((mag_data[5] * 256) + (mag_data[4] & 0xFE)) / 2;
  //mag[2] = ((mag_data[3]<<7) + (mag_data[2]>>1));

// End of modifications for Milo --------------------------

  if (mag[2] > 16383)
  {
    mag[2] -= 32768;
  }


}

/**
 * @brief Write I2C Data
 * @param [in] device_addr Device Address
 * @param [in] register_addr Write Register Address
 * @param [in] value Write Data
 */
void FaBo9Axis::writeI2c(byte device_addr, byte register_addr, byte value) {
  Wire.beginTransmission(device_addr);
  Wire.write(register_addr);
  Wire.write(value);
  Wire.endTransmission();
}

/**
 * @brief Read I2C Data
 * @param [in] device_addr Device Address
 * @param [in] register_addr Register Address
 * @param [in] num Data Length
 * @param [out] *buf Read Data
 */
void FaBo9Axis::readI2c(byte device_addr, byte register_addr, int num, byte *buf) {
  Wire.beginTransmission(device_addr);
  Wire.write(register_addr);
  Wire.endTransmission();

  //Wire.beginTransmission(DEVICE_ADDR);
  Wire.requestFrom((int)device_addr, num);

  int i = 0;
  while (Wire.available())
  {
    buf[i] = Wire.read();
    i++;
  }
  //Wire.endTransmission();
}