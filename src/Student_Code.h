#ifndef STUDENT_CODE_H
#define STUDENT_CODE_H

#define BLACKCOLOURTHRESHOLD 1500


#define MOTORDRIVEOFFSETKP 10
#define MOTORDRIVEOFFSETKI 0.001
#define ACCEPTABLEDISTANCERROR 10

void student_Main();    // The main entry point to the student code

// Add your function prototypes below
int convertPower(double powerLevel);
int convertEncoderCountToMilliMeters(int encoderCount);
void driveStraight(int distance);
void driveUntilDistanceTo(int distance);
void driveUntilBlack();





// DO NOT ADD ANY PROTOTYPES AFTER THIS LINE
#endif  // STUDENT_CODE_H