/******************************************************************************
  MotorTest.ino
  Serial Controlled Motor Driver
  Marshall Taylor @ SparkFun Electronics
  Sept 15, 2016
  https://github.com/sparkfun/Serial_Controlled_Motor_Driver
  https://github.com/sparkfun/SparkFun_Serial_Controlled_Motor_Driver_Arduino_Library

  Resources:
  Uses Wire.h for i2c operation
  Uses SPI.h for SPI operation

  Development environment specifics:
  Arduino IDE 1.6.7
  Teensy loader 1.27

  This code is released under the [MIT License](http://opensource.org/licenses/MIT).
  Please review the LICENSE.md file included with this example. If you have any questions
  or concerns with licensing, please contact techsupport@sparkfun.com.
  Distributed as-is; no warranty is given.
******************************************************************************/
//This example steps through all motor positions moving them forward, then backwards.
//To use, connect a redboard to the user port, as many slaves as desired on the expansion
//port, and run the sketch.
//
// Notes:
//    While using SPI, the defualt LEDPIN will not toggle
//    This steps through all 34 motor positions, which takes a few seconds to loop.

#include <Arduino.h>
#include <stdint.h>
#include "SCMD.h"
#include "SCMD_config.h" //Contains #defines for common SCMD register names and values
#include "Wire.h"

#define LEDPIN 13

//Global variables needed for PDM library
#define pdmDataBufferSize 4096 //Default is array of 4096 * 32bit
uint16_t pdmDataBuffer[pdmDataBufferSize];

//Global variables needed for the FFT in this sketch
float g_fPDMTimeDomain[pdmDataBufferSize * 2];
float g_fPDMFrequencyDomain[pdmDataBufferSize * 2];
float g_fPDMMagnitudes[pdmDataBufferSize * 2];
uint32_t sampleFreq;

//Enable these defines for additional debug printing
#define PRINT_PDM_DATA 0
#define PRINT_FFT_DATA 0
#define blinkPin LED_BUILTIN

#include <PDM.h> //Include PDM library included with the Aruino_Apollo3 core
AP3_PDM myPDM;   //Create instance of PDM class

//Math library needed for FFT
#define ARM_MATH_CM4
#include <arm_math.h>

#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1
float Cf; // Calibration factor
int Upperbound;
int Lowerbound;
int RobotSpeed;
int RightSpeed;
int LeftSpeed;
int Range;


SCMD myMotorDriver; //This creates the main object of one motor driver and connected slaves.

void setup()
{
  Serial.begin(115200);
  pinMode(LEDPIN, OUTPUT);

  Serial.println("LETS GO CRAZY.");

  //***** Configure the Motor Driver's Settings *****//
  //  .commInter face can be I2C_MODE or SPI_MODE
  myMotorDriver.settings.commInterface = I2C_MODE;
  //myMotorDriver.settings.commInterface = SPI_MODE;

  //  set address if I2C configuration selected with the config jumpers
  myMotorDriver.settings.I2CAddress = 0x5D; //config pattern "0101" on board for address 0x5A

  //  set chip select if SPI selected with the config jumpers
  myMotorDriver.settings.chipSelectPin = 10;

  //*****initialize the driver get wait for idle*****//
  while ( myMotorDriver.begin() != 0xA9 ) //Wait until a valid ID word is returned
  {
    Serial.println( "ID mismatch, trying again" );
    delay(500);
  }
  Serial.println( "ID matches 0xA9" );

  //  Check to make sure the driver is done looking for slaves before beginning
  Serial.print("Waiting for enumeration...");
  while ( myMotorDriver.ready() == false );
  Serial.println("Done.");
  Serial.println();

  //*****Set application settings and enable driver*****//

  while ( myMotorDriver.busy() );
  myMotorDriver.enable();

  Serial.println();

  // PDM
  if (myPDM.begin() == false) // Turn on PDM with default settings, start interrupts
  {
    Serial.println("PDM Init failed. Are you sure these pins are PDM capable?");
    while (1)
      ;
  }
  Serial.println("PDM Initialized");

  printPDMConfig();

}

void loop()
{
  //***** 5 - Fun Riding *****//

  //***** PDM *****//
  if (myPDM.available())
  {
    myPDM.getData(pdmDataBuffer, pdmDataBufferSize);

    GetFrequency(); // Read frequency and determine motion
  }

  // Go to Deep Sleep until the PDM ISR or other ISR wakes us.
  am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);

}

//*****************************************************************************
// Get Frequency Data
//*****************************************************************************
void GetFrequency(void)
{
  float fMaxValue;
  uint32_t ui32MaxIndex;
  int16_t *pi16PDMData = (int16_t *)pdmDataBuffer;
  uint32_t ui32LoudestFrequency;

  //
  // Convert the PDM samples to floats, and arrange them in the format
  // required by the FFT function.
  //
  for (uint32_t i = 0; i < pdmDataBufferSize; i++)
  {
    g_fPDMTimeDomain[2 * i] = pi16PDMData[i] / 1.0;
    g_fPDMTimeDomain[2 * i + 1] = 0.0;
  }

  //
  // Perform the FFT.
  //
  arm_cfft_radix4_instance_f32 S;
  arm_cfft_radix4_init_f32(&S, pdmDataBufferSize, 0, 1);
  arm_cfft_radix4_f32(&S, g_fPDMTimeDomain);
  arm_cmplx_mag_f32(g_fPDMTimeDomain, g_fPDMMagnitudes, pdmDataBufferSize);

  //
  // Find the frequency bin with the largest magnitude.
  //
  arm_max_f32(g_fPDMMagnitudes, pdmDataBufferSize / 2, &fMaxValue, &ui32MaxIndex);
  
  ui32LoudestFrequency = (sampleFreq * ui32MaxIndex) / pdmDataBufferSize;

  Serial.printf("Loudest frequency: %d         \n", ui32LoudestFrequency);

  if (ui32LoudestFrequency > 2000) {
    Robot_Goes_Crazy();
  }
  else {
    Robot_moves_slowly();
  }
}

//*****************************************************************************
// Robot goes crazy
//*****************************************************************************
void Robot_Goes_Crazy()
{
  digitalWrite( LEDPIN, LEFT_MOTOR );
  digitalWrite( LEDPIN, RIGHT_MOTOR );
  RobotSpeed = 255; Cf = 0.47;
  RightSpeed, LeftSpeed = CalibrateWheels(Cf, RobotSpeed); // Calibrate wheels

  myMotorDriver.setDrive( LEFT_MOTOR, 1, RightSpeed); // Go Crazy
  myMotorDriver.setDrive( RIGHT_MOTOR, 1, LeftSpeed);


}

//*****************************************************************************
// Robot Moves slowly
//*****************************************************************************
void Robot_moves_slowly()
{
  digitalWrite( LEDPIN, LEFT_MOTOR );
  digitalWrite( LEDPIN, RIGHT_MOTOR );
  RobotSpeed = 80;
  RightSpeed, LeftSpeed = CalibrateWheels(0.47, RobotSpeed); // Calibrate wheels

  myMotorDriver.setDrive( LEFT_MOTOR, 1, RightSpeed); //Move forward slowly
  myMotorDriver.setDrive( RIGHT_MOTOR, 0, LeftSpeed);

}


//*****************************************************************************
//
// Calibrate the wheels
//
//*****************************************************************************
int CalibrateWheels(float Cf, int RobotSpeed)
{
  //Cf - Calibration factor (1>0.5 - Lean right, 0<0.5 - Lean left)
  Upperbound = 255; Lowerbound = 50;

  if (RobotSpeed == Upperbound) { // Robot speed is Upperbound
    if (Cf < 0.5) { // Need to lean left
      RightSpeed = Upperbound;
      LeftSpeed = Upperbound + Upperbound * (Cf - 0.5);
    }
    else if (Cf > 0.5) { // Need to lean Right
      LeftSpeed = Upperbound;
      RightSpeed = Upperbound + Upperbound * (0.5 - Cf);
    }
  }
  else if (RobotSpeed == Lowerbound) { // Robot speed is Lowerbound

    if (Cf < 0.5) { // Need to lean left
      LeftSpeed = Lowerbound;
      RightSpeed = Lowerbound + Lowerbound * (0.5 - Cf);
    }
    else if (Cf > 0.5) { // Need to lean Right
      RightSpeed = Lowerbound;
      LeftSpeed = Lowerbound + Lowerbound * (Cf - 0.5);
    }
  }
  else if (RobotSpeed  < (Upperbound - Lowerbound) / 2) { // If less than middle of range

    Range = (RobotSpeed - Lowerbound);
    if (Cf < 0.5) { // Need to lean left

      LeftSpeed = RobotSpeed + Range * (Cf - 0.5);;
      RightSpeed = RobotSpeed + Range * (0.5 - Cf);
    }
    else if (Cf > 0.5) { // Need to lean Right
      RightSpeed = RobotSpeed + Range * (Cf - 0.5);;
      LeftSpeed = RobotSpeed + Range * (0.5 - Cf);
    }
  }
  else if (RobotSpeed  > (Upperbound - Lowerbound) / 2) { // If greater than middle of range

    Range = (Upperbound - RobotSpeed);
    if (Cf < 0.5) { // Need to lean left

      LeftSpeed = RobotSpeed + Range * (Cf - 0.5);;
      RightSpeed = RobotSpeed + Range * (0.5 - Cf);
    }
    else if (Cf > 0.5) { // Need to lean Right
      RightSpeed = RobotSpeed + Range * (Cf - 0.5);;
      LeftSpeed = RobotSpeed + Range * (0.5 - Cf);
    }
  }
  return RightSpeed, LeftSpeed;

}

//*****************************************************************************
//
// Print PDM configuration data.
//
//*****************************************************************************
void printPDMConfig(void)
{
  uint32_t PDMClk;
  uint32_t MClkDiv;
  float frequencyUnits;

  //
  // Read the config structure to figure out what our internal clock is set
  // to.
  //
  switch (myPDM.getClockDivider())
  {
    case AM_HAL_PDM_MCLKDIV_4:
      MClkDiv = 4;
      break;
    case AM_HAL_PDM_MCLKDIV_3:
      MClkDiv = 3;
      break;
    case AM_HAL_PDM_MCLKDIV_2:
      MClkDiv = 2;
      break;
    case AM_HAL_PDM_MCLKDIV_1:
      MClkDiv = 1;
      break;

    default:
      MClkDiv = 0;
  }

  switch (myPDM.getClockSpeed())
  {
    case AM_HAL_PDM_CLK_12MHZ:
      PDMClk = 12000000;
      break;
    case AM_HAL_PDM_CLK_6MHZ:
      PDMClk = 6000000;
      break;
    case AM_HAL_PDM_CLK_3MHZ:
      PDMClk = 3000000;
      break;
    case AM_HAL_PDM_CLK_1_5MHZ:
      PDMClk = 1500000;
      break;
    case AM_HAL_PDM_CLK_750KHZ:
      PDMClk = 750000;
      break;
    case AM_HAL_PDM_CLK_375KHZ:
      PDMClk = 375000;
      break;
    case AM_HAL_PDM_CLK_187KHZ:
      PDMClk = 187000;
      break;

    default:
      PDMClk = 0;
  }

  //
  // Record the effective sample frequency. We'll need it later to print the
  // loudest frequency from the sample.
  //
  sampleFreq = (PDMClk / (MClkDiv * 2 * myPDM.getDecimationRate()));

  frequencyUnits = (float)sampleFreq / (float)pdmDataBufferSize;

}
