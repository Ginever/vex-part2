#ifndef STUDENT_CODE_H
#define STUDENT_CODE_H

//Black and Brown colour thresholds
#define BLACKCOLOURTHRESHOLD 2390
#define BROWNCOLOURTHRESHOLD 1900

//Drive straight controller Ki and Kp
//Pulled here becuase they are used in more than one function
#define MOTORDRIVEOFFSETKP 2.5
#define MOTORDRIVEOFFSETKI 0.001

void student_Main();    // The main entry point to the student code

// Function prototypes below
int convertPower(double powerLevel);
int convertEncoderCountToMilliMeters(int encoderCount);
void driveStraight(int distance);
int driveUntilDistanceTo(int distance);
void driveUntilBlack();
int smoothAcceleration(int targetPower, int time);
void armPosition(int maxPower, int targetAngle);
void turnAngle(float targetPower, int targetAngle);
void advancedLineFollowing(float power);

// DO NOT ADD ANY PROTOTYPES AFTER THIS LINE
#endif  // STUDENT_CODE_H