#include "mathHelper.h"

#include <math.h>

bool checkAngle_greater(float angleA, float angleB) {
    float diff = angleA - angleB;
    if (diff < 0) {
        diff += 2 * M_PI;
    }
    return diff > M_PI;
}

bool checkAngle_lower(float angleA, float angleB) {
    float diff = angleB - angleA;
    if (diff < 0) {
        diff += 2 * M_PI;
    }
    return diff > M_PI;
}

float angles_add(float angleA, float angleB){
    float angle = angleA + angleB;
    if(angle > M_PI){
        angle -= 2*M_PI;
    } else if(angle < -M_PI){
        angle += 2*M_PI;
    }
    return angle;

}

float angle_subtract(float angleA, float angleB) {
    float diff = angleA - angleB;
    if (diff < -M_PI) {
        diff += 2 * M_PI;
    } else if (diff > M_PI) {
        diff -= 2 * M_PI;
    }
    return diff;
}

float getDistance(float x_0, float y_0, float x_1, float y_1){
    return (float) sqrt(pow((double)(x_1 - x_0), 2) + pow((double)(y_1 - y_0), 2));
}