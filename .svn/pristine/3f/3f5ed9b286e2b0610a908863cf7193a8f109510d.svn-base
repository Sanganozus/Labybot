#include "pose/pose.h"
#include "main.h"

#include <sensors/ISRCustom.h>
#include "pose.h"
#include "sensors/sensors.h"
#include <math.h>
#include <stdbool.h>
#include <tools/timeTask/timeTask.h>
#include <communication/communication.h>
#include "../tasks/taskManagement.h"


#include <avr/pgmspace.h>           // AVR Program Space Utilities
#include <stdlib.h> 

//#define MM_PER_TICK (0.140845070422535f)
#define MM_PER_TICK (45.0f*M_PI/1024.0f)

Pose_t pose;// = {0.0f, 0.0f, M_PI};
Pose_t * poseTemp;
bool poseUpdateFirst = true;



bool checkAprilPose = false;

float thetaTemp = 0;

float thetaDiff = 0.0f;

float distDiff = 0.0f;

uint16_t poseUpdateCounter = 0;


float deltaTotal(float deltaX, float deltaY){
    return sqrt(deltaX*deltaX + deltaY*deltaY);
}

//aktualisiert die Pose basierend auf Encoder-Werten
void poseUpdate() {
    if (checkAprilPose == false) {
        return;
    }


    int16_t encoder1;
    int16_t encoder2;
    getAndResetEncoders(&encoder1, &encoder2);

    float currRightMM = encoder1 * MM_PER_TICK;
    float currLeftMM = encoder2 * MM_PER_TICK;

    if (encoder1 != encoder2) {
        float deltaTheta = (currRightMM - currLeftMM) / (float) achsenlaenge;
        float deltaX = (currRightMM+currLeftMM) / (currRightMM-currLeftMM) * (achsenlaenge/2) * (sin(pose.theta + deltaTheta) - sin(pose.theta));
        float deltaY = (currRightMM+currLeftMM) / (currRightMM-currLeftMM) * (achsenlaenge/2) * (cos(pose.theta) - cos(pose.theta + deltaTheta));

        pose.x += deltaX;
        pose.y += deltaY;
        distDiff += deltaTotal(deltaX, deltaY);

        pose.theta += deltaTheta;
        thetaDiff += deltaTheta;

        if (pose.theta < -M_PI)
            pose.theta += 2*M_PI;

        if (pose.theta > M_PI)
            pose.theta -= 2*M_PI;
    } else {
        float d = (currRightMM + currLeftMM) / 2;
        pose.x += d * cos(pose.theta);
        pose.y += d * sin(pose.theta);
        //pose.theta += 0;
    }

    //updateCounts();
}

Pose_t *getPose() {
    return &pose;
}

bool firstAprilTagUpdate(){
    return checkAprilPose;
}

void poseUpdateAprilTag(const uint8_t* packet, __attribute__((unused)) const uint16_t size) {
    poseTemp = (Pose_t*) packet;
    
    if(logPose) communication_log_P(LEVEL_INFO, PSTR("Angeforderte POSE_APRIL_TAG: %i %i %i"), (int) poseTemp->x, (int) poseTemp->y, (int) (poseTemp->theta*100));
    
    aktualisierePose();
    unpauseTasks();

    if (!checkAprilPose) {
        checkAprilPose = true;
    }
}


//aktualisiere die Pose nur, wenn gerade kein Task ausgeführt wird!
void aktualisierePose() {
    if(logPose) {
        communication_log_P(LEVEL_INFO, PSTR("Pose  - April Tag neue Pose vorhanden (+)"));
        communication_log_P(LEVEL_INFO, PSTR("aktuelle Pose: %i %i %i"), (int)pose.x, (int)pose.y, (int)(pose.theta*100));
        communication_log_P(LEVEL_INFO, PSTR("neue Pose: %i %i %i"), (int)pose.x, (int)pose.y, (int)(pose.theta*100));
    }
    
    pose.x = poseTemp->x;
    pose.y = poseTemp->y;
    pose.theta = poseTemp->theta;
}



//function to return the April Tag Pose
void requestAprilTagPose(GetPose_t * aprilTag) {
    aprilTag->aprilTagType = APRIL_TAG_MAIN;
    communication_writePacket(CH_OUT_GET_POSE, (uint8_t*) aprilTag, sizeof(*aprilTag));
}


uint16_t requestAprilTagPoseCounter = 0;

void timeTask_RequestAprilTag() {
    TIMETASK(APRIL_TAG_TASK, 100) {
        requestAprilTagPoseCounter++;
        //if(requestAprilTagPoseCounter >= 255 && !isTaskActive()) {
        if(requestAprilTagPoseCounter >= 120 && !isTaskActive()) {
            pauseTasks();
            GetPose_t * requestPoseAprilTag = (GetPose_t*) malloc(sizeof(GetPose_t));
            requestAprilTagPose(requestPoseAprilTag);
            free(requestPoseAprilTag);
            requestAprilTagPoseCounter = 0;
        }
    }
}

void startMeasuring_thetaDiff() {
    thetaDiff = 0.0f;
}

float getThetaDiff(){
    return thetaDiff;
}

void startMeasuring_distDiff() {
    distDiff = 0.0f;
}

float getDistDiff() {
    return distDiff;
}

Direction_t pose_getCurrentCardinalDirection() {
    //Theta gibt den Winkel zwischen der x-Achse und der Vorwärtsrichtung des Roboters an
    if(pose.theta >= -M_PI_4 && pose.theta <= M_PI_4) { //Theta zwischen -pi/4 und pi/4
        return DIRECTION_EAST;
    } else if(pose.theta > M_PI_4 && pose.theta <= 3*M_PI_4) { //Theta zwischen pi/4 und 3pi/4
        return DIRECTION_NORTH;
    } else if((pose.theta > -3*M_PI_4) && (pose.theta <= -M_PI_4)) { //Theta zwischen 3pi/4 und 5pi/4
        return DIRECTION_SOUTH;
    } else if(pose.theta > 3*M_PI_4 || pose.theta < -3*M_PI_4) { //Theta zwischen 5pi/4 und 7pi/4
        return DIRECTION_WEST;
    } 
    communication_log(LEVEL_SEVERE, "pose_getCurrentCardinalDirection() -> theta out of range");
    return DIRECTION_NORTH;
}

float getAngle_forCardinalDirection(Direction_t direction) {
    switch(direction) {
        case DIRECTION_NORTH:
            return M_PI_2;
        case DIRECTION_EAST:
            return 0.0f;
        case DIRECTION_SOUTH:
            return -M_PI_2;
        case DIRECTION_WEST:
            return M_PI;
    }
    communication_log(LEVEL_SEVERE, "getAngle_forCardinalDirection() -> direction out of range");
    return 0.0f;
}

void correctPose(Direction_t direction, float correctionValue){
    if(logPoseCorrection) communication_log_P(LEVEL_INFO, PSTR(""));
    if(logPoseCorrection) communication_log_P(LEVEL_INFO, PSTR("----- correctPose. direction: %i, correctionValue: %.3f -----"), direction, correctionValue);
    switch(direction){
        case DIRECTION_NORTH:
            pose.x = correctionValue;
            if(logPoseCorrection) communication_log_P(LEVEL_INFO, PSTR("  -> pose.x = correctionValue. pose.x: %.3f"), pose.x);
            break;
        case DIRECTION_EAST:
            pose.y = correctionValue;
            if(logPoseCorrection) communication_log_P(LEVEL_INFO, PSTR("  -> pose.y = correctionValue. pose.y: %.3f"), pose.y);
            break;
        case DIRECTION_SOUTH:
            pose.x = correctionValue;
            if(logPoseCorrection) communication_log_P(LEVEL_INFO, PSTR("  -> pose.x = correctionValue. pose.x: %.3f"), pose.x);
            break;
        case DIRECTION_WEST:
            pose.y = correctionValue;
            if(logPoseCorrection) communication_log_P(LEVEL_INFO, PSTR("  -> pose.y = correctionValue. pose.y: %.3f"), pose.y);
            break;
    }
}

/*void correctPose(Direction_t direction, float correctionValue){
    if(logPoseCorrection) communication_log_P(LEVEL_INFO, PSTR(""));
    if(logPoseCorrection) communication_log_P(LEVEL_INFO, PSTR("----- correctPose. direction: %i, correctionValue: %.3f -----"), direction, correctionValue);
    switch(direction){
        case DIRECTION_NORTH:
            pose.x += correctionValue;
            if(logPoseCorrection) communication_log_P(LEVEL_INFO, PSTR("  -> pose.x += correctionValue. pose.x: %.3f"), pose.x);
            break;
        case DIRECTION_EAST:
            pose.y += correctionValue;
            if(logPoseCorrection) communication_log_P(LEVEL_INFO, PSTR("  -> pose.y += correctionValue. pose.y: %.3f"), pose.y);
            break;
        case DIRECTION_SOUTH:
            pose.x -= correctionValue;
            if(logPoseCorrection) communication_log_P(LEVEL_INFO, PSTR("  -> pose.x -= correctionValue. pose.x: %.3f"), pose.x);
            break;
        case DIRECTION_WEST:
            pose.y -= correctionValue;
            if(logPoseCorrection) communication_log_P(LEVEL_INFO, PSTR("  -> pose.y -= correctionValue. pose.y: %.3f"), pose.y);
            break;
    }
}*/












