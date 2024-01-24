#include <inttypes.h>
#include <motor/motor.h>

#include <tools/timeTask/timeTask.h>
#include <communication/communication.h>
#include <avr/pgmspace.h>
#include <stdlib.h>
#include <math.h>

#include "driving.h"
#include "../pose/pose.h"
#include "../explorer/robot.h"
#include "../helper/mathHelper.h"
#include "main.h"



//----- Globale Variablen -----//
// Driving
int speed_Left = 0;
int speed_Right = 0;

// Balancing
static float tolerance_theta = M_PI_2/45.0f; //Grad 2 Toleranz


static bool balancing_flag = 0;
static Direction_t balancing_direction = DIRECTION_NORTH;
static float balancing_fixedValue = 0;
static float balancing_fixedTheta = 0.0;


void setToleranceValues(uint16_t tol_fixedValue, uint16_t tol_theta, uint16_t corrValue){
    tolerance_fixedValue = (float)tol_fixedValue;
    tolerance_theta = M_PI_2/(90.0f/((float)tol_theta));
    correctionValue = 1.0f + (float)corrValue/100.0f;
    communication_log_P(LEVEL_INFO, PSTR("tolerance_fixedValue: %i, tolerance_theta: %i, correctionValue: %i"), (int)tolerance_fixedValue, (int)(tolerance_theta*100), (int)(correctionValue*100));
}

void setMotorSpeed(int speedLeft, int speedRight) {
    //if(logBalancing) communication_log_P(LEVEL_INFO, PSTR("left: %i, right: %i", speedLeft, speedRight);
    Motor_setPWM(-speedLeft, -speedRight);
}

void startBalancing(Direction_t dir, float fixedValue){
    if(logBalancing) communication_log_P(LEVEL_INFO, PSTR("startBalancing with current Direction: %i"), dir);

    balancing_fixedValue = fixedValue;
    balancing_direction = dir;
    balancing_flag = 1;
}

void stopBalancing(){
    balancing_flag = 0;
}

/**
 * Generiert einen correctionValue für balancingTheta, welcher beim default correctionValue startet und, falls der momentane 
 * getPose()->Theta sich in die richtige Richtung verändert (größer werden: falls er kleiner tolerance_fixedTheta ist. kleiner werden: falls er größer tolerance_fixedTheta ist)
 * 
 * Wenn 
*/

void initDrive(int speed, int steering){
    speed_Left = speed;
    speed_Right = speed - 2 * steering;
    setMotorSpeed(speed_Left, speed_Right);
}

/**
 * Nur in Kombination mit stopBalancing() verwenden !!!!!!!!!!!!!!
*/
void initDrive_withBalancing_withPars(Direction_t direction, int speed, int steering, float fixedValue, float fixedTheta) {
    speed_Left = speed;
    speed_Right = speed - 2 * steering;
    if(speed_Left == speed_Right){
        startBalancing(direction, fixedValue);
    }
    setMotorSpeed(speed_Left, speed_Right);

    balancing_fixedTheta = fixedTheta;
}

void initDrive_withBalancing(Direction_t currentDir, int speed, int steering){
    float fixVal = 0.0f;

    switch(currentDir){
        case DIRECTION_NORTH:
        case DIRECTION_SOUTH:
            fixVal = getPose()->x;
            break;
        case DIRECTION_EAST:
        case DIRECTION_WEST:
            fixVal = getPose()->y;
            break;
    }
    initDrive_withBalancing_withPars(currentDir, speed, steering, fixVal, getAngle_forCardinalDirection(currentDir));
} 

void initDrive_withBalancing_withFixedValue(Direction_t currentDir, int speed, int steering, float fixedValue){
    initDrive_withBalancing_withPars(currentDir, speed, steering, fixedValue, getAngle_forCardinalDirection(currentDir));
}

void stopDrive() {
    speed_Left = 0;
    speed_Right = 0;
    setMotorSpeed(speed_Left, speed_Right);
    stopBalancing();
}

int getSpeed_left() {
    return speed_Left;
}

int getSpeed_right() {
    return speed_Right;
}

uint16_t balancingDebugCounter = 0;

void correction_moreToLeft(float value){
    if(logBalancing && balancingDebugCounter % 50 == 0) communication_log_P(LEVEL_INFO, PSTR("Correction more to left. Left: %i, Right: %i"), (uint16_t)speed_Left, (uint16_t)(speed_Right * correctionValue));
    setMotorSpeed(speed_Left, speed_Right * value);
}

void correction_moreToRight(float value){   
    if(logBalancing && balancingDebugCounter % 50 == 0) communication_log_P(LEVEL_INFO, PSTR("Correction more to right. Left: %i, Right: %i"), (uint16_t)(speed_Left * correctionValue), (uint16_t)speed_Right);
    setMotorSpeed(speed_Left * value, speed_Right);
}

void correction_remove(){
    if(logBalancing && balancingDebugCounter % 50 == 0) communication_log_P(LEVEL_INFO, PSTR("Correction remove"));
    setMotorSpeed(speed_Left, speed_Right);
}




void balance_coordinates(Direction_t dir, float coordinateToCheck){
    float leftBorder = balancing_fixedValue - tolerance_fixedValue;
    float rightBorder = balancing_fixedValue + tolerance_fixedValue;


    char *dirCoordinateString;
    if(dir == DIRECTION_NORTH || dir == DIRECTION_SOUTH){
        dirCoordinateString = "x";
    } else {
        dirCoordinateString = "y";
    }

    if(logBalancing && balancingDebugCounter % 20 == 0) communication_log_P(LEVEL_INFO, PSTR("balance_coordinates - direction: %s, coordinateToCheck: %i, fixedValue: %i, left border: %i, right border: %i. coordinateToCheck>rightBorder: %i"), cardStr(dir), (int)coordinateToCheck, (int)balancing_fixedValue, (int)leftBorder, (int)rightBorder, (int)((int)coordinateToCheck>(int)rightBorder));
    switch(dir) {
        case DIRECTION_NORTH:
        case DIRECTION_WEST:
            if(((int)coordinateToCheck) > ((int)rightBorder)) {
                if(logBalancing && balancingDebugCounter % 50 == 0) communication_log_P(LEVEL_INFO, PSTR("%s zu groß => More to left. coordinateToCheck: %i, rightBorder: %i"), dirCoordinateString, (int)coordinateToCheck, (int)rightBorder);
                correction_moreToLeft(((correctionValue - 1.0f)/2) + 1.0f);
            } else {
                if(logBalancing && balancingDebugCounter % 50 == 0) communication_log_P(LEVEL_INFO, PSTR("%s zu klein => More to right. coordinateToCheck: %i, leftBorder: %i"), dirCoordinateString, (int)coordinateToCheck, (int)leftBorder);
                correction_moreToRight(((correctionValue - 1.0f)/2) + 1.0f);
            }
            break;
        case DIRECTION_EAST:
        case DIRECTION_SOUTH:
            if(((int)coordinateToCheck) > ((int)rightBorder)) {
                if(logBalancing && balancingDebugCounter % 50 == 0) communication_log_P(LEVEL_INFO, PSTR("%s zu groß => More to right. coordinateToCheck: %i, leftBorder: %i"), dirCoordinateString, (int)coordinateToCheck, (int)leftBorder);
                correction_moreToRight(((correctionValue - 1.0f)/2) + 1.0f);
            } else {
                if(logBalancing && balancingDebugCounter % 50 == 0) communication_log_P(LEVEL_INFO, PSTR("%s zu klein => More to left. coordinateToCheck: %i, rightBorder: %i"), dirCoordinateString, (int)coordinateToCheck, (int)rightBorder);
                correction_moreToLeft(((correctionValue - 1.0f)/2) + 1.0f);
            }
            break;
    }
    
}

void balance_theta(){
    float currentTheta = getPose()->theta;
    float targetTheta = balancing_fixedTheta;

    float leftBorder = targetTheta - tolerance_theta;
    float rightBorder = targetTheta + tolerance_theta;

    if(logBalancing && balancingDebugCounter % 20 == 0) communication_log_P(LEVEL_INFO, PSTR("Theta: %i, balancing_fixedTheta: %i, left border: %i, right border: %i"), (int)(getPose()->theta*100), (int)(balancing_fixedTheta*100), (int)(leftBorder)*100, (int)(rightBorder)*100);
    if(checkAngle_lower(currentTheta, leftBorder)){
        if(logBalancing && balancingDebugCounter % 20 == 0) communication_log_P(LEVEL_INFO, PSTR("Theta zu klein"));

        correction_moreToRight(correctionValue);
    } else if(checkAngle_greater(currentTheta, rightBorder)) {
        if(logBalancing && balancingDebugCounter % 20 == 0) communication_log_P(LEVEL_INFO, PSTR("Theta zu groß"));

        correction_moreToLeft(correctionValue);
    } else {
        if(logBalancing && balancingDebugCounter % 20 == 0) communication_log_P(LEVEL_INFO, PSTR("Theta in Toleranz"));

        correction_remove();
    }
}

void balance_switch(Direction_t dir, float coordinateToCheck){
    if(logBalancing && balancingDebugCounter % 50 == 0) communication_log_P(LEVEL_INFO, PSTR("balance_switch - coordinateToCheck: %i, fixedValue: %i, left border: %i, right border: %i"), (int)coordinateToCheck, (int)balancing_fixedValue, (int)(balancing_fixedValue-tolerance_fixedValue), (int)(balancing_fixedValue+tolerance_fixedValue));
        
    if(coordinateToCheck >= balancing_fixedValue - tolerance_fixedValue && coordinateToCheck <= balancing_fixedValue + tolerance_fixedValue){
        if(logBalancing && balancingDebugCounter % 50 == 0) communication_log_P(LEVEL_INFO, PSTR("Balancing in direction %s inside tolerance_fixedValue"), cardStr(dir));
        balance_theta();
    } else if(coordinateToCheck > balancing_fixedValue + tolerance_fixedValue || coordinateToCheck < balancing_fixedValue - tolerance_fixedValue) {
        if(logBalancing && balancingDebugCounter % 50 == 0) communication_log_P(LEVEL_INFO, PSTR("Balancing in direction %s outside tolerance_fixedValue"), cardStr(dir));
        balance_coordinates(dir, coordinateToCheck);
    } else {
        if(logBalancing && balancingDebugCounter % 50 == 0) communication_log_P(LEVEL_INFO, PSTR("Balancing in direction %s no balancing"), cardStr(dir));
    }
}

void checkBalancing(){
    TIMETASK(BALANCE_TASK, 50) {
        balancingDebugCounter++;
        if(balancing_flag == 1) {
            switch(balancing_direction){
                case DIRECTION_NORTH: // x konstant
                case DIRECTION_SOUTH:
                    balance_switch(balancing_direction, getPose()->x);
                    break;
                case DIRECTION_EAST: // y konstant
                case DIRECTION_WEST:
                    balance_switch(balancing_direction, getPose()->y);
                    break;
            }
        }
    }
}