#include <stdio.h>
#include <stdbool.h>
#define PI 3.14159265359

int drivingWheelDiameter = 70;

double convertEncoderCountToMilliMeters(int encoderCount){
    //Using arc=pi*radius in radians
    return (PI*encoderCount*drivingWheelDiameter)/900;
}

float convertMillimetersToEncoderCount(double millimeters) {
    return (900*millimeters)/(PI*drivingWheelDiameter);
}

int main() {
    for (int x = 0; x < 10000;  x++){
        if (convertMillimetersToEncoderCount(convertEncoderCountToMilliMeters(x)) != x){
            printf("Something went wrong:\n");
            printf("x:  %d\n", x);
            printf("Millimeter Value: %f\n", convertEncoderCountToMilliMeters(x));
            printf("Calculated encoder value: %f", convertMillimetersToEncoderCount(convertEncoderCountToMilliMeters(x)));
            break;
        }
    }
    if (false != (1 == 2)){
        printf("Hi");
    }
    return 0;
}