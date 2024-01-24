#include <stdbool.h>
#include <stdint.h>
#include "io/adc/adc.h"
#include <communication/communication.h>
#include <avr/pgmspace.h> 

#include "ISRCustom.h"
#include "vision.h"
#include "sensors.h"
#include "main.h"


//------------------------------------------------------------

uint16_t getDistance_toWall_forward() {
    if(!(convertInfraredToMM(ADC_getFilteredValue(2)) < 120)) {
        return -1;
    }
    return convertInfraredToMM(ADC_getFilteredValue(2));
}

uint16_t getDistance_toWall_right() {
    if(!(convertInfraredToMM(ADC_getFilteredValue(0)) < 120)) {
        return -1;
    }

    return convertInfraredToMM(ADC_getFilteredValue(0));
}

uint16_t getDistance_toWall_left() {
    if(!(convertInfraredToMM(ADC_getFilteredValue(1)) < 120)) {
        return -1;
    }
    return convertInfraredToMM(ADC_getFilteredValue(1));
}


uint16_t getDistanceToWall(RobotDirection_t dir) {
    switch(dir) {
        case FORWARD:
            return getDistance_toWall_forward();
        case RIGHT: 
            return getDistance_toWall_right();
        case LEFT:
            return getDistance_toWall_left();
        default:
            return false;
    }
}


bool isWall_forward() {
    if (getDistanceToWall(FORWARD) < 120) {
        return true;
    }
    return false;
}

bool isWall_right() {
    if (getDistanceToWall(RIGHT) < 120) {
        return true;
    }
    return false;
}

bool isWall_left() {
    if (getDistanceToWall(LEFT) < 120) {
        return true;
    }
    return false;
}

bool isWall(RobotDirection_t dir) {
    switch(dir) {
        case FORWARD:
            return isWall_forward();
        case RIGHT: 
            return isWall_right();
        case LEFT:
            return isWall_left();
        default:
            return false;
    }
}


float poseCorrectionValue_withWalls(){
    if(logPoseCorrection) communication_log_P(LEVEL_INFO, PSTR(""));
    if(logPoseCorrection) communication_log_P(LEVEL_INFO, PSTR("----- poseCorrectionValue_withWalls -----"));

    if(isWall_forward() //Nicht zwingend notwendig, geht aber sicher, dass man nach vorne fährt
    || !isWall_right() || !isWall_left()){
        return -1;
    }

    uint16_t left = getDistance_toWall_left();
    uint16_t right = getDistance_toWall_right();
    if(logPoseCorrection) communication_log_P(LEVEL_INFO, PSTR("  -> left: %i, right: %i"), left, right);

    float center = (float)(left + right) / 2.0;
    if(logPoseCorrection) communication_log_P(LEVEL_INFO, PSTR("  -> center: %.3f"), center);

    float distanceToCenter = center - (float)left;
    if(logPoseCorrection) communication_log_P(LEVEL_INFO, PSTR("  -> distanceToCenter: %.3f"), distanceToCenter);

    return distanceToCenter;
}   

