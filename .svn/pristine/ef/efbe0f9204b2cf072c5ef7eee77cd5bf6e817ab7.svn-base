#include "taskMethods.h"

#include "../driving/driving.h"
#include "../pose/pose.h"

#include <communication/packetTypes.h>


//----- Task Methods -----//

/**
 * Uses Balancing
*/
void driveForward(Parameters* par) {
    initDrive_withBalancing(par->direction, par->speed, 0);
}

void driveForward_withFixedValue(Parameters* par) {
    initDrive_withBalancing_withFixedValue(par->direction, par->speed, 0, par->fixedValue);
}

/**
 * Uses Balancing
*/
void driveBackwards(Parameters* par) {
    initDrive_withBalancing(par->direction, -(int)(par->speed), 0);
}

/**
 * Doesn't use Balancing
*/
void rotate_clockwise(Parameters* par) {
    initDrive(par->speed, par->speed);
}

/**
 * Doesn't use Balancing
*/
void rotate_counterClockwise(Parameters* par) {
    initDrive(-(int)(par->speed), -(int)(par->speed));
}


//----- Abort Methods -----//

void abortDriving(){
    stopDrive();
}

TaskMethod getMethod_driveForward(){
    return (TaskMethod) &driveForward;
}

TaskMethod getMethod_driveForward_withFixedValue(){
    return (TaskMethod) &driveForward_withFixedValue;
}

TaskMethod getMethod_driveBackwards(){
    return (TaskMethod) &driveBackwards;
}

TaskMethod getMethod_rotate_clockwise(){
    return (TaskMethod) &rotate_clockwise;
}

TaskMethod getMethod_rotate_counterClockwise(){
    return (TaskMethod) &rotate_counterClockwise;
}