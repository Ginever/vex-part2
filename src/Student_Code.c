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
int drivingWheelDiameter = 103;	    // diameter of the driving wheels [mm]
int robotWidth = 250;				// width of the robot including the wheel thickness [mm]
int wheelWidth = 22;				// width of the driving wheel [mm]
double armRatio = 0.0;				// ratio of arm shaft rotations to arm motor shaft rotations
double encCountPerRev = 0.0;	    // number of encoder ticks per 1 revolution of the motor shaft
// ------------------------------------------------------------------------------------

bool wasRightWheelForwards = 1;
bool wasLeftWheelForwards = 1;

/* Write your code in the function below. You may add helper functions below the studentCode function. */
void student_Main()
{   
    //Config
    armUp(5000);
    resetEncoder(ArmEncoder);

    lineFollow(40);

    // armPosition(60,-10);
    
    // driveUntilDistanceTo(250);

    // armUp(5000);
    
    // turnAngle(50, -90);

    // driveStraight(350);

    // turnAngle(50,90);

    // driveStraight(1500);

    // turnAngle(50, -90);


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
    return (PI*encoderCount*drivingWheelDiameter)/900;
}

// __ [ DRIVE STRAIGHT ] ______________________________________________
/**
 * @brief commands the robot to drive straight for a specifed distance 
 * using a PI controller
 * @param distance the target distance for the robot to drive in millimeters
 */
void driveStraight(int distance){
    int distanceError, motorOffsetError, power;
    int totalMotorOffsetError = 0;
    int totalDistanceError = 0;

    float u;
    float Kp = 1;
    float Ki = 0.0001;

    resetEncoder(LeftEncoder);
    resetEncoder(RightEncoder);
    resetTimer(T_1);
    lcd_clear();
    do {
        distanceError = distance - convertEncoderCountToMilliMeters((readSensor(LeftEncoder) + readSensor(RightEncoder))/2);
        totalDistanceError = totalDistanceError + distanceError;

        motorOffsetError = convertEncoderCountToMilliMeters(readSensor(LeftEncoder) - readSensor(RightEncoder));
        totalMotorOffsetError = totalMotorOffsetError + motorOffsetError;

        power = saturate(Kp*distanceError + Ki*totalDistanceError, -80,80);
        u = MOTORDRIVEOFFSETKP*motorOffsetError + MOTORDRIVEOFFSETKI*totalMotorOffsetError;

        motorPower(LeftMotor,  convertPower(smoothAcceleration(power - u, readTimer(T_1))));
        motorPower(RightMotor, convertPower(smoothAcceleration(power + u, readTimer(T_1))));

        lcd_print(LCDLine1, "driveStraight: ");
        lcd_print(LCDLine2, "left: %d right: %d", convertEncoderCountToMilliMeters(readSensor(LeftEncoder)), convertEncoderCountToMilliMeters(readSensor(RightEncoder)));
        lcd_print(LCDLine3, "power: %d", power);
        lcd_print(LCDLine4, "error: %d, %d", distanceError, totalDistanceError);
        delay(50); //sample at 20Hz
    } while (abs(distanceError) > 5);

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
int smoothDriveSigma(int leftTargetPower, int rightTargetPower, int time, int attack){
    double p = saturate((float)time/attack,0,1);


    lcd_print(LCDLine7, "%f", p);

    motorPower(LeftMotor, leftTargetPower * p);
    motorPower(RightMotor, rightTargetPower * p);

}

int smoothAcceleration(int targetPower, int time){
    double attack = 10*targetPower;
    lcd_print(LCDLine6, "Acceleareat: %f", saturate(time/attack, 0, 1));
    return targetPower*saturate(time/attack, 0, 1);
}


void armPosition(int maxPower, int targetAngle) {
    int error,armPower;
    int Kp = 20;
   
    do {
        error = (double)targetAngle - ((double)readSensor(ArmEncoder)) * (360.0/(7.0*900.0)) - 52.0;

        armPower = saturate(Kp * error, -1 * maxPower, maxPower);
        armPower = convertPower(armPower);
       
        motorPower(ArmMotor, armPower);

        lcd_print(LCDLine1, "Error is %d", error);
        lcd_print(LCDLine2, "Encoder is %d", readSensor(ArmEncoder));
        delay(50);
    } while (abs(error) > 1);
   
    motorPower(ArmMotor, 0);  
}

// void turnAngle(int targetAngle){
//     int distanceError, motorOffsetError, power;
//     int totalMotorOffsetError = 0;
//     int totalDistanceError = 0;

//     float u;
//     float Kp = 1;
//     float Ki = 0.0001;

//     resetEncoder(LeftEncoder);
//     resetEncoder(RightEncoder);
//     resetTimer(T_1);
//     lcd_clear();
//     do {
//         distanceError = distance - convertEncoderCountToMilliMeters((readSensor(LeftEncoder) + readSensor(RightEncoder))/2);
//         totalDistanceError = totalDistanceError + distanceError;

//         motorOffsetError = convertEncoderCountToMilliMeters(readSensor(LeftEncoder) - readSensor(RightEncoder));
//         totalMotorOffsetError = totalMotorOffsetError + motorOffsetError;

//         power = saturate(Kp*distanceError + Ki*totalDistanceError, -80,80);
//         u = MOTORDRIVEOFFSETKP*motorOffsetError + MOTORDRIVEOFFSETKI*totalMotorOffsetError;

//         motorPower(LeftMotor,  convertPower(smoothAcceleration(power - u, readTimer(T_1))));
//         motorPower(RightMotor, convertPower(smoothAcceleration(power + u, readTimer(T_1))));

//         lcd_print(LCDLine1, "driveStraight: ");
//         lcd_print(LCDLine2, "left: %d right: %d", convertEncoderCountToMilliMeters(readSensor(LeftEncoder)), convertEncoderCountToMilliMeters(readSensor(RightEncoder)));
//         lcd_print(LCDLine3, "power: %d", power);
//         lcd_print(LCDLine4, "error: %d, %d", distanceError, totalDistanceError);
//         delay(50); //sample at 20Hz
//     } while (abs(distanceError) > 1);

//     motorPower(LeftMotor, 0);
//     motorPower(RightMotor, 0);
// }

void turnAngleRadius(float targetPower, int targetAngle, int targetRadiusOfCurvature){
    int currentAngle, currentRadiusOfCurvature, rightDistance, leftDistance, angleError, radiusOfCurvatureError, uCurvature;
    float rightPower;
    float leftPower;

    int totalAngleError = 0;
    int totalRadiusOfCurvatureError = 0;
    int angleKp = 10;
    int curvatureKp = 30;
    float angleKi = 0;
    float curvatureKi = 0;

    int turnRadius = robotWidth - wheelWidth;

    resetEncoder(LeftEncoder);
    resetEncoder(RightEncoder);
    resetTimer(T_1);
    lcd_clear();

    do {
        //get sensed wheel displacments
        leftDistance = convertEncoderCountToMilliMeters(readSensor(LeftEncoder));
        rightDistance = convertEncoderCountToMilliMeters(readSensor(RightEncoder));

        //Calculate current angle and raduis of curvature
        currentAngle = ((float)(rightDistance - leftDistance)/turnRadius)*180.0/PI;
        currentRadiusOfCurvature = ((float)turnRadius*(leftDistance + rightDistance))/(2.0*(rightDistance - leftDistance));

        //Angle errors
        angleError = targetAngle - currentAngle;
        totalAngleError = totalAngleError + angleError;
    	
        //Radius of Curvature errors
        radiusOfCurvatureError = targetRadiusOfCurvature - currentRadiusOfCurvature;
        totalRadiusOfCurvatureError = totalRadiusOfCurvatureError + radiusOfCurvatureError;

        //saturate power
        //! This might cause problems in the future 
        rightPower = saturate(angleKp*angleError + angleKi*totalAngleError, -1*targetPower, targetPower);

        //adjust left power so the robot attempts to follow target raduis of curvature
        leftPower = rightPower*((float)(targetRadiusOfCurvature-(turnRadius/2.0))/(targetRadiusOfCurvature+(turnRadius/2.0)));

        uCurvature = curvatureKi*radiusOfCurvatureError + curvatureKi*totalRadiusOfCurvatureError;

        //Adjust for curvature errrors
        rightPower = rightPower - uCurvature;
        leftPower = leftPower + uCurvature;

        lcd_print(LCDLine1, "turnwithRadius: ");
        lcd_print(LCDLine2, "left: %d right: %d", convertEncoderCountToMilliMeters(readSensor(LeftEncoder)), convertEncoderCountToMilliMeters(readSensor(RightEncoder)));
        lcd_print(LCDLine3, "powers: %f, %f", leftPower, rightPower);
        lcd_print(LCDLine4, "Derror: %d, %d", angleError, totalAngleError);
        lcd_print(LCDLine5, "Rerror: %d, %d", radiusOfCurvatureError, totalRadiusOfCurvatureError);
        lcd_print(LCDLine6, "uCurvature: %d", uCurvature);

        motorPower(LeftMotor, convertPower(leftPower));
        motorPower(RightMotor, convertPower(rightPower));

        delay(50); // sample at 20Hz
    } while (abs(angleError) > 1);

    motorPower(LeftMotor, 0);
    motorPower(RightMotor, 0);
}

/**
 * @brief a function to turn a user specifed angle
 * @param targetPoewr target turn power (%)
*/
void turnAngle(float targetPower, int targetAngle){
    int currentAngle, rightDistance, leftDistance, angleError, radiusOfCurvatureError, uPower, uCurvature;

    int totalAngleError = 0;
    int totalRadiusOfCurvatureError = 0;
    int angleKp = 5;
    int curvatureKp = 4;
    float angleKi = 0;
    float curvatureKi = 0;

    resetEncoder(LeftEncoder);
    resetEncoder(RightEncoder);
    resetTimer(T_1);
    lcd_clear();

    do {
        leftDistance = convertEncoderCountToMilliMeters(readSensor(LeftEncoder));
        rightDistance = convertEncoderCountToMilliMeters(readSensor(RightEncoder));

        currentAngle = ((float)(rightDistance - leftDistance)/(robotWidth - wheelWidth))*180.0/PI;

        //Angle errors
        angleError = targetAngle - currentAngle;
        totalAngleError = totalAngleError + angleError;

        radiusOfCurvatureError = leftDistance - rightDistance;
        totalRadiusOfCurvatureError = totalRadiusOfCurvatureError + radiusOfCurvatureError;

        uPower = saturate(angleKp*angleError + angleKi*totalAngleError, targetPower*-1, targetPower);

        uPower = convertPower(uPower);

        uCurvature = curvatureKi*radiusOfCurvatureError + curvatureKi*totalRadiusOfCurvatureError;

        lcd_print(LCDLine1, "turnAroundCenter: ");
        lcd_print(LCDLine2, "left: %d right: %d", convertEncoderCountToMilliMeters(readSensor(LeftEncoder)), convertEncoderCountToMilliMeters(readSensor(RightEncoder)));
        lcd_print(LCDLine3, "power: %d", uPower);
        lcd_print(LCDLine4, "Derror: %d, %d", angleError, totalAngleError);
        lcd_print(LCDLine5, "Rerror: %d, %d", radiusOfCurvatureError, totalRadiusOfCurvatureError);
        lcd_print(LCDLine6, "uCurvature: %d", uCurvature);

        motorPower(LeftMotor, -1*uPower - uCurvature);
        motorPower(RightMotor, uPower + uCurvature);

        delay(50); // sample at 20Hz
    } while (abs(angleError) > 2);

    motorPower(LeftMotor, 0);
    motorPower(RightMotor, 0);
}

//https://prod.liveshare.vsengsaas.visualstudio.com/join?F6DA46744063F53C464336479CDFADE246E0


// return 2 if black, 1 of brown, 0 if not black/brown
int lightColour(int sensorReading) {   
    if (sensorReading > BLACKCOLOURTHRESHOLD) {
        return 2;
    }
    if (sensorReading > BROWNCOLOURTHRESHOLD) {
        return 1;
    }
    return 0; 
}

void lineFollow(int inputPower) {
    int leftLightSensor, midLightSensor, rightLightSensor;
    int black = 0;

    int power = convertPower(inputPower);

    motorPower(LeftMotor, power);
    motorPower(RightMotor, -1 * power);

    delay(200);

    motorPower(LeftMotor, 0);
    motorPower(RightMotor, 0);
    
    lcd_clear();

    while (black == 0) {
        leftLightSensor = lightColour(readSensor(LeftLight));
        midLightSensor = lightColour(readSensor(MidLight));
        rightLightSensor = lightColour(readSensor(RightLight));

        lcd_print(LCDLine1, "Left: %d, %d", readSensor(LeftLight), leftLightSensor);
        lcd_print(LCDLine2, "Mid: %d, %d", readSensor(MidLight), midLightSensor);
        lcd_print(LCDLine3, "Right: %d, %d", readSensor(RightLight), rightLightSensor);
        
        if (leftLightSensor == 2 || midLightSensor == 2 || rightLightSensor == 2) {
            black == 1;
            break;
        }

        if (!leftLightSensor && !midLightSensor && !rightLightSensor) {
    

        }
        else if (!leftLightSensor && !midLightSensor && rightLightSensor) {
            motorPower(LeftMotor, power);
            motorPower(RightMotor, -1 * power);
        }
        else if (!leftLightSensor && midLightSensor && !rightLightSensor) {
            motorPower(LeftMotor, power);
            motorPower(RightMotor, power);
        }
        else if (!leftLightSensor && midLightSensor && rightLightSensor) {
            motorPower(LeftMotor, power); // will change
            motorPower(RightMotor, power);
        }
        else if (leftLightSensor && !midLightSensor && !rightLightSensor) {
            motorPower(LeftMotor, -1 * power);
            motorPower(RightMotor, power);
        }
        else if (leftLightSensor && !midLightSensor && rightLightSensor) {
            motorPower(LeftMotor, power);
            motorPower(RightMotor, power); //will change
        }
        else if (leftLightSensor && midLightSensor && !rightLightSensor) {
            motorPower(LeftMotor, power);
            motorPower(RightMotor, power); //will change
            
        }
        else if (leftLightSensor && midLightSensor && rightLightSensor) {
            motorPower(LeftMotor, power);
            motorPower(RightMotor, -1 * power); // will change
        }
        delay(50);
    }
}
