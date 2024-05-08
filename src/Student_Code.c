/**
 * @file Student_Code.c
 * @author Lachlan Dean & Zac Ginever
 * @brief the main file for the robot control code to allow the robot to successfully navigate the warehouse
 * @version 1.0
 * @date 08/05/24
 * @copyright Copyright (c) 2024
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
int drivingWheelDiameter = 103; // diameter of the driving wheels [mm]
int robotWidth = 250;           // width of the robot including the wheel thickness [mm]
int wheelWidth = 22;            // width of the driving wheel [mm]
double armRatio = 7.0;          // ratio of arm shaft rotations to arm motor shaft rotations
double encCountPerRev = 900;    // number of encoder ticks per 1 revolution of the motor shaft
// ------------------------------------------------------------------------------------

void student_Main()
{
    // Config !! DO NOT REMOVE !!
    armUp(5000);
    resetEncoder(ArmEncoder);
    resetTimer(T_4);
    // End of Config

    //pick up payload
    int payloadDistance = driveUntilDistanceTo(380);
    armPosition(60, -12.2);
    driveStraight(100);
    armUp(5000);
    driveStraight(-1 * payloadDistance - 40);
    delay(50);

    // navigate to line
    turnAngle(60, 92);
    delay(50);
    driveStraight(720);
    delay(50);
    turnAngle(60, -92);
    delay(50);
    driveUntilBlack();

    // follow line
    advancedLineFollowing(40);
    delay(50);

    // drop off payload
    driveUntilDistanceTo(425);
    delay(50);
    armPosition(60, 0);

    // pause to stabilise the payload to increase placment accuracy
    delay(500);
    armPosition(60, -12.2);

    //reverse away from payload
    delay(50);
    driveStraight(-100);
    delay(50);
    armUp(5000);
    delay(50);

    // return home
    driveStraight(200);
    delay(50);
    turnAngle(60, -90);
    delay(50);
    driveStraight(1300);
    delay(50);
    turnAngle(60, -90);
    delay(50);
    driveStraight(370);

    lcd_print(LCDLine7, "Timer: %d", readTimer(T_4));
}

// __[ CONVERT POWER ]________________________________________________
/**
// @brief Coverts a percentage (-100 to 100) to a motor output power
// where 100% is 5000 mV and -100 is 5000 mV
// @param powerLevel a floating point percentage between 100 and -100
// @return an integer voltage in millivolts
*/
int convertPower(double powerLevel)
{
    // Limit input to between -100 and 100
    double saturatedPowerLevel = saturate(powerLevel, -100, 100);

    // voltage calculation
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
int convertEncoderCountToMilliMeters(int encoderCount)
{
    // Using arc=pi*radius in radians
    return (PI * encoderCount * drivingWheelDiameter) / encCountPerRev;
}

// __ [ DRIVE STRAIGHT ] ______________________________________________
/**
 * @brief commands the robot to drive straight for a specifed distance
 * using a PI controller
 * @param distance the target distance for the robot to drive in millimeters
 */
void driveStraight(int distance)
{
    int distanceError, motorOffsetError, previousError;
    int totalMotorOffsetError = 0;
    int totalDistanceError = 0;

    float u, power;
    float Kp = 1.3;
    float Ki = 0.2;

    // reset encoders and timers
    resetEncoder(LeftEncoder);
    resetEncoder(RightEncoder);
    resetTimer(T_1);
    resetTimer(T_2);

    do
    {
        // Calulate error e(t) = r(t) - u(t)
        distanceError = distance - convertEncoderCountToMilliMeters((readSensor(LeftEncoder) + readSensor(RightEncoder)) / 2);

        // anti-wind up measure
        // totalDistanceError will not integrate if distanceError is already saturating the control effort
        if (abs(distanceError * Kp) < 80)
        {
            // Calculate intergral error
            totalDistanceError = totalDistanceError + distanceError;
        }

        // Calculate motor offset error to allow the robot to drive straight
        motorOffsetError = convertEncoderCountToMilliMeters(readSensor(LeftEncoder) - readSensor(RightEncoder));
        totalMotorOffsetError = totalMotorOffsetError + motorOffsetError;

        // Calculate drive effort
        power = saturate((double)Kp * distanceError + (double)Ki * totalDistanceError, -80, 80);

        // Calculate drive straight control effort
        u = MOTORDRIVEOFFSETKP * motorOffsetError + MOTORDRIVEOFFSETKI * totalMotorOffsetError;

        // Apply acceleration smoothing and convert power to mV before sending the power levels to the motors
        motorPower(LeftMotor, convertPower(smoothAcceleration(power - u, readTimer(T_1))));
        motorPower(RightMotor, convertPower(smoothAcceleration(power + u, readTimer(T_1))));

        // reset timer if power changes between samples
        if (distanceError != previousError)
            resetTimer(T_2);
        previousError = distanceError;

        delay(50); // sample at 20Hz
    } while (abs(totalDistanceError) > 120 || abs(distanceError) > 5);

    // Stop drive motors
    motorPower(LeftMotor, 0);
    motorPower(RightMotor, 0);
}

// __ [ DRIVE UNTIL DISTANCE TO ] _____________________________________
/**
 * @brief Drive the robot forward until it is at a specifed distance away
 * from the object in front of it
 * @param distance how far away from the object the robot should stop (mm)
 * @return the distance the robot has driven (mm)
 */
int driveUntilDistanceTo(int distance)
{
    // calculate distance the robot needs to drive based on sonar distance
    int driveDistance = readSensor(SonarSensor) - distance;

    // Drive calculated distance
    driveStraight(driveDistance);

    return driveDistance;
}

// __ [ DRIVE UNTIL BLACK ] ___________________________________________
/**
 * @brief Drives the robot forward in a straight line until any light sensor
 * detects a black line
 * @param none
 */
void driveUntilBlack()
{
    int motorOffsetError;
    int totalMotorOffsetError = 0;
    int power = 50;
    float u;

    // reset timers and encoders
    resetEncoder(LeftEncoder);
    resetEncoder(RightEncoder);
    resetTimer(T_1);

    do
    {
        // Calculate motor offset error to allow the robot to drive straight
        motorOffsetError = convertEncoderCountToMilliMeters(readSensor(LeftEncoder) - readSensor(RightEncoder));

        // Calculate the integral of the motor offset error
        totalMotorOffsetError = totalMotorOffsetError + motorOffsetError;

        // Calculate straightening control effot
        u = MOTORDRIVEOFFSETKP * motorOffsetError + MOTORDRIVEOFFSETKI * totalMotorOffsetError;

        // Apply smooth acceleration, convert the power to millivolts and send it to the motors
        motorPower(LeftMotor, convertPower(smoothAcceleration(power - u, readTimer(T_1))));
        motorPower(RightMotor, convertPower(smoothAcceleration(power + u, readTimer(T_1))));

        delay(50); // sample at 20 Hz

        // Exit if any light sensor detects black
    } while (readSensor(LeftLight) < BLACKCOLOURTHRESHOLD && readSensor(MidLight) < BLACKCOLOURTHRESHOLD && readSensor(RightLight) < BLACKCOLOURTHRESHOLD);

    // Stop drive motors
    motorPower(LeftMotor, 0);
    motorPower(RightMotor, 0);

    // required delay for saftey
    delay(1000);
}

// __ [ SMOOTH ACCELERATION ] _________________________________________
/**
 * @brief limits the rate of acceleration to prevent slip by reducing the power
 * level sent to the wheel based on the time provided
 * @param targetPower The power level the robot is demanding
 * @param time The time since the power started being sent to the wheels
 * @return A percentage power that has been smoothed
 */
int smoothAcceleration(int targetPower, int time)
{
    return targetPower * saturate((double)time / (10.0 * abs(targetPower)), 0, 1);
}

// __ [ ARM POSITION] _________________________________________________
/**
 * @brief Moves the arm to the specifed target angle
 * @param maxPower The maximum power the arm motor is allowed to demand (%)
 * @param targetAngle The angle the arm is required to go to (°)
 */
void armPosition(int maxPower, int targetAngle)
{
    int error, armPower;
    int Kp = 20;

    do
    {
        // calculate angle error
        error = (double)targetAngle - ((double)readSensor(ArmEncoder)) * (360.0 / (armRatio * encCountPerRev)) - 52.0;

        // Calculate control effort
        armPower = saturate(Kp * error, -1 * maxPower, maxPower);

        // convert power to mV
        armPower = convertPower(armPower);

        // Set motor power to calculated power
        motorPower(ArmMotor, armPower);

        delay(50); // sample at 50 Hz
    } while (abs(error) > 1);

    motorPower(ArmMotor, 0);
}

/**
 * @brief a function to turn a user specifed angle
 * @param targetPower target turn power (%)
 * @param targetAngle Target angle for the robot to turn (°)
 */
void turnAngle(float targetPower, int targetAngle)
{
    int currentAngle, rightDistance, leftDistance, angleError, radiusOfCurvatureError, uPower, uCurvature;

    int totalAngleError = 0;
    int totalRadiusOfCurvatureError = 0;
    int angleKp = 5;
    int curvatureKp = 4;
    float angleKi = 0.1;
    float curvatureKi = 0;

    // reset encoders and timers
    resetEncoder(LeftEncoder);
    resetEncoder(RightEncoder);
    resetTimer(T_1);

    do
    {
        // Get wheel displacements
        leftDistance = convertEncoderCountToMilliMeters(readSensor(LeftEncoder));
        rightDistance = convertEncoderCountToMilliMeters(readSensor(RightEncoder));

        // Calculate current angle based on wheel displacements
        currentAngle = ((float)(rightDistance - leftDistance) / (robotWidth - wheelWidth)) * 180.0 / PI;

        // Calculate angle error
        angleError = targetAngle - currentAngle;

        // Anti-wind measure up to prevent integrator overshoot
        if (abs(angleKp * angleError) < targetPower)
        {
            // Summing integral error
            totalAngleError = totalAngleError + angleError;
        }

        // Calculate radius of curvature error
        radiusOfCurvatureError = leftDistance - rightDistance;
        totalRadiusOfCurvatureError = totalRadiusOfCurvatureError + radiusOfCurvatureError;

        // calculate control effort for angle displacement
        uPower = saturate(angleKp * angleError + angleKi * totalAngleError, targetPower * -1, targetPower);
        // convert power to millivolts
        uPower = convertPower(uPower);

        // calculate control effort for radius of curvature
        uCurvature = curvatureKi * radiusOfCurvatureError + curvatureKi * totalRadiusOfCurvatureError;

        // Apply control efforts to the motors
        motorPower(LeftMotor, -1 * uPower - uCurvature);
        motorPower(RightMotor, uPower + uCurvature);

        delay(50); // sample at 20Hz
    } while (abs(angleError) > 1);

    // Stop drive motors
    motorPower(LeftMotor, 0);
    motorPower(RightMotor, 0);
}

// __ [ ADVANCED LINE FOLLOWING ] ______________________________________
/**
 *  @brief Uses edge following to follow the line with more accuracy
 *  @param power the power the robot is targeted to drive at (%)
 */
void advancedLineFollowing(float power)
{
    int leftLight, midLight, rightLight, u, lightError;

    int targetLightLevel = 1800;
    float Kp = 1.5;
    bool isTurning = false;

    power = convertPower(power);

    // Stop drive motors
    motorPower(LeftMotor, 0);
    motorPower(RightMotor, 0);
    resetTimer(T_1);
    resetTimer(T_2);

    while (true)
    {
        delay(50);

        // read Light sensor values
        leftLight = readSensor(LeftLight);
        midLight = readSensor(MidLight);
        rightLight = readSensor(RightLight);

        // exit the loop in any sensor detects a black line
        if ((leftLight > BLACKCOLOURTHRESHOLD || midLight > BLACKCOLOURTHRESHOLD || rightLight > BLACKCOLOURTHRESHOLD) && readTimer(T_2) > 500) break;
        
        //to prevent the random turn at the end of the line following section that makes the robot innacurate
        if ((leftLight > BROWNCOLOURTHRESHOLD && rightLight > BROWNCOLOURTHRESHOLD - 200) || (rightLight > BROWNCOLOURTHRESHOLD && leftLight > BROWNCOLOURTHRESHOLD - 200)) continue;
        

        // Turn Right (CW) slowly if right sensor detects line and the robot turned right less than 1.5 secs ago
        if (rightLight > BROWNCOLOURTHRESHOLD && readTimer(T_1) < 800 && readTimer(T_1) > 300)
        {
            motorPower(LeftMotor, power);
            motorPower(RightMotor, 0);

            isTurning = false;
            continue;
        }

        // turn Right (CW) if right light sensor detects brown
        if (rightLight > BROWNCOLOURTHRESHOLD)
        {
            motorPower(LeftMotor, power);
            motorPower(RightMotor, -power);

            isTurning = false;

            if (readTimer(T_1) > 300) resetTimer(T_1);
            continue;
        }

        // contiue turning left (CCW) if already turning left
        // prevents the edge detector detecting the wrong edge
        if (isTurning) continue;
    	
        // turn left (CCW) if left light sensor detects brown
        if (leftLight > BROWNCOLOURTHRESHOLD)
        {
            motorPower(LeftMotor, -power);
            motorPower(RightMotor, power);

            isTurning = true;
            continue;
        }

        // Edge following controller to keep robot on edge of line
        lightError = targetLightLevel - midLight;

        //Calculate light error
        u = Kp * lightError;

        //Apply light error and send power to motors
        motorPower(LeftMotor, power + u);
        motorPower(RightMotor, power - u);
    };

    // Stop drive motors
    motorPower(LeftMotor, 0);
    motorPower(RightMotor, 0);
}