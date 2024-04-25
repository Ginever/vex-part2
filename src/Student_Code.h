#ifndef STUDENT_CODE_H
#define STUDENT_CODE_H

#define BLACKCOLOURTHRESHOLD 2390
#define BROWNCOLOURTHRESHOLD 1900


#define MOTORDRIVEOFFSETKP 2
#define MOTORDRIVEOFFSETKI 0.001
#define ACCEPTABLEDISTANCERROR 10

void student_Main();    // The main entry point to the student code

// Add your function prototypes below
int convertPower(double powerLevel);
int convertEncoderCountToMilliMeters(int encoderCount);
void driveStraight(int distance);
int driveUntilDistanceTo(int distance);
void driveUntilBlack();
int smoothAcceleration(int targerPower, int time);
void armPosition(int maxPower, int targetAngle);
void turnAngleRadius(float targetPower, int targetAngle, int targetRadiusOfCurvature);
void turnAngle(float targetPower, int targetAngle);
void lineFollow(int inputPower);
void advancedLineFollowing(float power);

// DO NOT ADD ANY PROTOTYPES AFTER THIS LINE
#endif  // STUDENT_CODE_H