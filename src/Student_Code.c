/**
 * @file Student_Code.c
 * @author Lachlan Dean & Zac Ginever
 * @brief description of this file
 * @version 0.1
 * @date 22-03-24
 *
 * @copyright Copyright (c) 2024
 *
 */

/* Libraries. DO NOT REMOVE */
#include "main.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "Student_Code.h"

// ---------------------- Defining physical robot parameters --------------------------
// Update these numbers to match the physical robot (information found in the lab manual)
int drivingWheelDiameter = 70;	    // diameter of the driving wheels [mm]
int robotWidth = 0;					// width of the robot including the wheel thickness [mm]
int wheelWidth = 0;					// width of the driving wheel [mm]
double drivingWheelRatio = 0.0;	    // ratio of wheel shaft rotations to wheel motor shaft rotations
double armRatio = 0.0;				// ratio of arm shaft rotations to arm motor shaft rotations
double encCountPerRev = 0.0;	    // number of encoder ticks per 1 revolution of the motor shaft
// ------------------------------------------------------------------------------------

/* Write your code in the function below. You may add helper functions below the studentCode function. */
void student_Main()
{   
    armUp(5000);
    driveStraight(1000);
}

// __[ CONVERT POWER ]________________________________________________
/**
// @brief Coverts a percentage (-100 to 100) to a motor output power
// where 100% is 5000 mV and -100 is 5000 mV
// @param powerLevel a floating point percentage between 100 and -100
// @return an integer voltage in millivolts
*/
int convertPower(double powerLevel){
    //Limit input to between -100 and 100
    double saturatedPowerLevel = saturate(powerLevel, -100, 100);

    //voltage calculation
    int voltagePower = 5000 * (saturatedPowerLevel / 100);
    
    return voltagePower;
}

// __ [ CONVERT ENCODER COUNT TO MILLIMETERS ] ________________________
/**
 * @brief converts a drive motor encoder value to a distance traveled
 * in millimeters
 * @param encoderCount the encoder count to be translated into millimeters
 * @return A distance value that the wheel has traveled in millimeters 
 * assuming no slip
*/
int convertEncoderCountToMilliMeters(int encoderCount){
    //Using arc=pi*radius in radians
    return (drivingWheelDiameter/2)*(2*PI*encoderCount/900);
}

// __ [ DRIVE STRAIGHT ] ______________________________________________
/**
 * @brief commands the robot to drive straight for a specifed distance 
 * using a PI controller
 * @param distance the target distance for the robot to drive in millimeters
 */
void driveStraight(int distance){
    int acceptableError = 5;

    int distanceError = distance;
    int motorOffsetError = 0;
    float Kp = 10;
    int power = 4000;

    resetEncoder(LeftEncoder);
    resetEncoder(RightEncoder);
    lcd_clear();

    while (distanceError > acceptableError){
        distanceError = distance - convertEncoderCountToMilliMeters(readSensor(LeftEncoder));
        motorOffsetError = motorOffsetError + readSensor(LeftEncoder) - readSensor(RightEncoder);

        power = convertPower(saturate(Kp*distanceError, -40,40));

        motorPower(LeftMotor, power);
        motorPower(RightMotor, power*(1+ motorOffsetError*MOTORDRIVEOFFSETKI) + MOTORDRIVEOFFSETKP*(readSensor(LeftEncoder) - readSensor(RightEncoder)));

        lcd_print(LCDLine1, "driveStraight: ");
        lcd_print(LCDLine2, "left: %d right: %d", convertEncoderCountToMilliMeters(readSensor(LeftEncoder)), convertEncoderCountToMilliMeters(readSensor(RightEncoder)));
        lcd_print(LCDLine3, "power: %d", power);
        delay(50);
    }

    motorPower(LeftMotor, 0);
    motorPower(RightMotor, 0);
}

// __ [ DRIVE UNTIL DISTANCE TO ] _____________________________________
/**
 * @brief Drive the robot forward until it is at a specifed distance away
 * from the object in front of it
 * @param distance how far away from the object the robot should stop (mm)
*/
void driveUntilDistanceTo(int distance){
    int driveDistance = readSensor(SonarSensor) - distance;

    driveStraight(driveDistance);
}

// __ [ DRIVE UNTIL BLACK ] ___________________________________________
/**
 * @brief Drives the robot forward in a straight line until any light sensor
 * detects a black line
 * @param none
*/
void driveUntilBlack() {
    int motorOffsetError = 0;
    int power = convertPower(50);

    resetEncoder(LeftEncoder);
    resetEncoder(RightEncoder);
    lcd_clear();

    while (readSensor(LeftLight) < BLACKCOLOURTHRESHOLD && readSensor(MidLight) < BLACKCOLOURTHRESHOLD && readSensor(RightLight) < BLACKCOLOURTHRESHOLD){
        motorOffsetError = motorOffsetError + readSensor(LeftEncoder) - readSensor(RightEncoder);

        motorPower(LeftMotor, power);
        motorPower(RightMotor, power*(1+ motorOffsetError*MOTORDRIVEOFFSETKI) + MOTORDRIVEOFFSETKP*(readSensor(LeftEncoder) - readSensor(RightEncoder)));

        lcd_print(LCDLine1, "driveUntilBlack");
        lcd_print(LCDLine2, "left: %d right: %d", convertEncoderCountToMilliMeters(readSensor(LeftEncoder)), convertEncoderCountToMilliMeters(readSensor(RightEncoder)));
        lcd_print(LCDLine3, "Light Levels: %d %d %d", readSensor(LeftLight), readSensor(MidLight), readSensor(RightLight));
        lcd_print(LCDLine4, "power: %d", power);
        delay(50);
    }

    motorPower(LeftMotor, 0);
    motorPower(RightMotor, 0);
}

// __ [ SMOOTH DRIVE ] ________________________________________________
/**
 * @brief A function that smooths the acceleration of the robot in order
 * to minimize wheel slipping
 * @param leftTargetPower The target power of the left wheel (%)
 * @param rightTargetPower The target power of the right wheel (%)
 * @param time the time since this function was lasted called
 * @return int 
 */
int smoothDrive(int leftTargetPower, int rightTargetPower, int time){
    return 0;
}