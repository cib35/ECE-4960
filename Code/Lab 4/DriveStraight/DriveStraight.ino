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

SCMD myMotorDriver; //This creates the main object of one motor driver and connected slaves.

void setup()
{
  Serial.begin(9600);
  pinMode(LEDPIN, OUTPUT);

  Serial.println("Starting sketch.");

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

}

#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1
float Cf; // Calibration factor
int Upperbound;
int Lowerbound;
int RobotSpeed;
int RightSpeed;
int LeftSpeed;
int Range;

void loop()
{
  //***** 4(c) - Move Straight Forward *****//

  Serial.println("Moving Forward");
  
    digitalWrite( LEDPIN, LEFT_MOTOR );
    digitalWrite( LEDPIN, RIGHT_MOTOR );  
    Cf = 0.47; // Calibration factor (1>0.5 - Lean right, 0<0.5 - Lean left)
    Upperbound = 255; Lowerbound = 50; 
    RobotSpeed = 100; 

    // Scale speed
    Serial.print("Robot Speed:"); Serial.print(RobotSpeed);
    Serial.println();
    Serial.print("Upper Bound:"); Serial.print(Upperbound);
    Serial.println();
    Serial.print("Lower Bound:"); Serial.print(Lowerbound);
    Serial.println();  
    Serial.println(RobotSpeed==Upperbound);
    Serial.println();  

    
    if (RobotSpeed == Upperbound){ // Robot speed is Upperbound
      Serial.print("1");
      if (Cf < 0.5) { // Need to lean left
        RightSpeed = Upperbound; 
        LeftSpeed = Upperbound + Upperbound*(Cf-0.5);
      }
      else if (Cf > 0.5) { // Need to lean Right
        LeftSpeed = Upperbound; 
        RightSpeed = Upperbound + Upperbound*(0.5-Cf);
      }
    }
    else if (RobotSpeed == Lowerbound){ // Robot speed is Lowerbound
            Serial.print("2");

      if (Cf < 0.5) { // Need to lean left
        LeftSpeed = Lowerbound; 
        RightSpeed = Lowerbound + Lowerbound*(0.5-Cf);
      }
      else if (Cf > 0.5) { // Need to lean Right
        RightSpeed = Lowerbound; 
        LeftSpeed = Lowerbound + Lowerbound*(Cf-0.5);
      }
    }
    else if (RobotSpeed  < (Upperbound - Lowerbound)/2) { // If less than middle of range
            Serial.print("3");

      Range = (RobotSpeed-Lowerbound);
      if (Cf < 0.5) { // Need to lean left
      
        LeftSpeed = RobotSpeed + Range*(Cf-0.5);; 
        RightSpeed = RobotSpeed + Range*(0.5-Cf);
      }
      else if (Cf > 0.5) { // Need to lean Right
        RightSpeed = RobotSpeed + Range*(Cf-0.5);; 
        LeftSpeed = RobotSpeed + Range*(0.5-Cf);
      }
    }
    else if (RobotSpeed  > (Upperbound - Lowerbound)/2) { // If greater than middle of range
            Serial.print("4");

      Range = (Upperbound-RobotSpeed);
      if (Cf < 0.5) { // Need to lean left
      
        LeftSpeed = RobotSpeed + Range*(Cf-0.5);; 
        RightSpeed = RobotSpeed + Range*(0.5-Cf);
      }
      else if (Cf > 0.5) { // Need to lean Right
        RightSpeed = RobotSpeed + Range*(Cf-0.5);; 
        LeftSpeed = RobotSpeed + Range*(0.5-Cf);
      }
    }
    Serial.println();
    Serial.print("Right Speed:"); Serial.print(RightSpeed);
    Serial.println();
    Serial.print("Left Speed:"); Serial.print(LeftSpeed);
    Serial.println();
    
    myMotorDriver.setDrive( LEFT_MOTOR, 1, RightSpeed); //Drive Left wheel forward at Speed Computed above
    myMotorDriver.setDrive( RIGHT_MOTOR, 0, LeftSpeed); //Drive Right wheel forward at Speed Computed above
    delay(2500); // 6 feet is distance
    myMotorDriver.setDrive( LEFT_MOTOR, 1, 0); // Stop Left
    myMotorDriver.setDrive( RIGHT_MOTOR, 1, 0); // Stop Right
    delay(9999);
    

}
