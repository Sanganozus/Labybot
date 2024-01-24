#include "../explorer/robot.h"
#include "../explorer/labyrinthState.h"
#include "../sensors/vision.h"
#include "../helper/mathHelper.h"

#include <communication/communication.h>
#include <math.h>

char *angleTest(){
    if(checkAngle_greater(0.0f, -3*M_PI_4) != false){
        return "angleTest - ERROR: checkAngle_greater(0.0f, -3*M_PI_4) != false";
    }
    if(checkAngle_greater(M_PI_4, M_PI_2) != true){
        return "angleTest - ERROR: checkAngle_greater(M_PI_4, M_PI_2) != true";
    }
    if(checkAngle_greater(3*M_PI_4, -3*M_PI_4) != true){
        return "angleTest - ERROR: checkAngle_greater(3*M_PI_4, -3*M_PI_4) != true";
    }
    if(checkAngle_greater(M_PI_2, -M_PI_2) != false){ //Sonderfall: 180 Grad
        return "angleTest - ERROR: checkAngle_greater(M_PI_2, -M_PI_2) != false";
    }
    if(checkAngle_greater(M_PI, -3*M_PI_4) != true){
        return "angleTest - ERROR: checkAngle_greater(M_PI, -3*M_PI_4) != true";
    }

    if(checkAngle_lower(0.0f, -3*M_PI_4) != true){
        return "angleTest - ERROR: checkAngle_lower(0.0f, -3*M_PI_4) != true";
    }
    if(checkAngle_lower(M_PI_4, M_PI_2) != false){
        return "angleTest - ERROR: checkAngle_lower(M_PI_4, M_PI_2) != false";
    }
    if(checkAngle_lower(3*M_PI_4, -3*M_PI_4) != false){
        return "angleTest - ERROR: checkAngle_lower(3*M_PI_4, -3*M_PI_4) != false";
    }
    if(checkAngle_lower(M_PI_2, -M_PI_2) != false){ //Sonderfall: 180 Grad
        return "angleTest - ERROR: checkAngle_lower(M_PI_2, -M_PI_2) != false";
    }
    if(checkAngle_lower(M_PI, -3*M_PI_4) != false){
        return "angleTest - ERROR: checkAngle_lower(M_PI, -3*M_PI_4) != false";
    }

    return "angleTest - FINE";
}

char *totalOrientationTest(){
    //----- Robot Test -----//
    //Test Total Orientation
    if(getTotalOrientation(0, DIRECTION_NORTH) != DIRECTION_NORTH){
        return "totalOrientationTest - ERROR: getTotalOrientation(FORWARD, DIRECTION_NORTH) != DIRECTION_NORTH";
    }
    if(getTotalOrientation(0, DIRECTION_EAST) != DIRECTION_EAST){
        return "totalOrientationTestTest - ERROR: getTotalOrientation(FORWARD, DIRECTION_EAST) != DIRECTION_EAST";
    }
    if(getTotalOrientation(0, DIRECTION_SOUTH) != DIRECTION_SOUTH){
        return "totalOrientationTest - ERROR: getTotalOrientation(FORWARD, DIRECTION_SOUTH) != DIRECTION_SOUTH";
    }
    if(getTotalOrientation(0, DIRECTION_WEST) != DIRECTION_WEST){
        return "totalOrientationTest - ERROR: getTotalOrientation(FORWARD, DIRECTION_WEST) != DIRECTION_WEST";
    }

    if(getTotalOrientation(1, DIRECTION_NORTH) != DIRECTION_EAST){
        return "totalOrientationTest - ERROR: getTotalOrientation(RIGHT, DIRECTION_NORTH) != DIRECTION_EAST";
    }
    if(getTotalOrientation(1, DIRECTION_EAST) != DIRECTION_SOUTH){
        return "totalOrientationTest - ERROR: getTotalOrientation(RIGHT, DIRECTION_EAST) != DIRECTION_SOUTH";
    }
    if(getTotalOrientation(1, DIRECTION_SOUTH) != DIRECTION_WEST){
        return "totalOrientationTest - ERROR: getTotalOrientation(RIGHT, DIRECTION_SOUTH) != DIRECTION_WEST";
    }
    if(getTotalOrientation(1, DIRECTION_WEST) != DIRECTION_NORTH){
        return "totalOrientationTest - ERROR: getTotalOrientation(RIGHT, DIRECTION_WEST) != DIRECTION_NORTH";
    }

    if(getTotalOrientation(2, DIRECTION_NORTH) != DIRECTION_SOUTH){
        return "totalOrientationTest - ERROR: getTotalOrientation(BACKWARD, DIRECTION_NORTH) != DIRECTION_SOUTH";
    }
    if(getTotalOrientation(2, DIRECTION_EAST) != DIRECTION_WEST){
        return "totalOrientationTest - ERROR: getTotalOrientation(BACKWARD, DIRECTION_EAST) != DIRECTION_WEST";
    }
    if(getTotalOrientation(2, DIRECTION_SOUTH) != DIRECTION_NORTH){
        return "totalOrientationTest - ERROR: getTotalOrientation(BACKWARD, DIRECTION_SOUTH) != DIRECTION_NORTH";
    }
    if(getTotalOrientation(2, DIRECTION_WEST) != DIRECTION_EAST){
        return "totalOrientationTest - ERROR: getTotalOrientation(BACKWARD, DIRECTION_WEST) != DIRECTION_EAST";
    }

    if(getTotalOrientation(3, DIRECTION_NORTH) != DIRECTION_WEST){
        return "totalOrientationTest - ERROR: getTotalOrientation(LEFT, DIRECTION_NORTH) != DIRECTION_WEST";
    }
    if(getTotalOrientation(3, DIRECTION_EAST) != DIRECTION_NORTH){
        return "totalOrientationTest - ERROR: getTotalOrientation(LEFT, DIRECTION_EAST) != DIRECTION_NORTH";
    }
    if(getTotalOrientation(3, DIRECTION_SOUTH) != DIRECTION_EAST){
        return "totalOrientationTest - ERROR: getTotalOrientation(LEFT, DIRECTION_SOUTH) != DIRECTION_EAST";
    }
    if(getTotalOrientation(3, DIRECTION_WEST) != DIRECTION_SOUTH){
        return "totalOrientationTest - ERROR: getTotalOrientation(LEFT, DIRECTION_WEST) != DIRECTION_SOUTH";
    }

    return "totalOrientationTest - FINE";
}

char *test_getTileCoordinates(){
    if(getTile_x(1) != -3.0f*LABY_CELLSIZE){
        return "getTileCoordinatesTest - ERROR: getTile_x(1) != -3*LABY_CELLSIZE";
    }
    if(getTile_x(2) != -2.0f*LABY_CELLSIZE){
        return "getTileCoordinatesTest - ERROR: getTile_x(2) != -2*LABY_CELLSIZE";
    }
    if(getTile_x(3) != -LABY_CELLSIZE){
        return "getTileCoordinatesTest - ERROR: getTile_x(3) != -LABY_CELLSIZE";
    }
    if(getTile_x(4) != 0 ){
        return "getTileCoordinatesTest - ERROR: getTile_x(4) != 0";
    }
    if(getTile_x(5) != LABY_CELLSIZE){
        return "getTileCoordinatesTest - ERROR: getTile_x(5) != LABY_CELLSIZE";
    }
    if(getTile_x(6) != 2.0f*LABY_CELLSIZE){
        return "getTileCoordinatesTest - ERROR: getTile_x(6) != 2*LABY_CELLSIZE";
    }
    if(getTile_x(7) != 3.0f*LABY_CELLSIZE){
        return "getTileCoordinatesTest - ERROR: getTile_x(7) != 3*LABY_CELLSIZE";
    }

    if(getTile_y(1) != 3.0f*LABY_CELLSIZE){
        return "getTileCoordinatesTest - ERROR: getTile_y(1) != 3*LABY_CELLSIZE";
    }
    if(getTile_y(2) != 2.0f*LABY_CELLSIZE){
        return "getTileCoordinatesTest - ERROR: getTile_y(2) != 2*LABY_CELLSIZE";
    }
    if(getTile_y(3) != LABY_CELLSIZE){
        return "getTileCoordinatesTest - ERROR: getTile_y(3) != LABY_CELLSIZE";
    }
    if(getTile_y(4) != 0){
        return "getTileCoordinatesTest - ERROR: getTile_y(4) != 0";
    }
    if(getTile_y(5) != -LABY_CELLSIZE){
        return "getTileCoordinatesTest - ERROR: getTile_y(5) != -LABY_CELLSIZE";
    }
    if(getTile_y(6) != -2.0f*LABY_CELLSIZE){
        return "getTileCoordinatesTest - ERROR: getTile_y(6) != -2*LABY_CELLSIZE";
    }
    if(getTile_y(7) != -3.0f*LABY_CELLSIZE){
        return "getTileCoordinatesTest - ERROR: getTile_y(7) != -3*LABY_CELLSIZE";
    }

    return "getTileCoordinatesTest - FINE";
}

/*char *isExitTest(){
    if(isExit(1, 1, DIRECTION_NORTH, true, false, false) != FORWARD){
        return "isExitTest - ERROR: isExit(1, 1, DIRECTION_NORTH, true, false, false) != FORWARD";
    }
    if(isExit(1, 1, DIRECTION_NORTH, false, false, true) != LEFT){
        return "isExitTest - ERROR: isExit(1, 1, DIRECTION_NORTH, false, false, true) != LEFT";
    }
    if(isExit(1, 1, DIRECTION_NORTH, false, true, false) != -1){
        return "isExitTest - ERROR: isExit(1, 1, DIRECTION_NORTH, false, true, false) != -1";
    }

    return "isExitTest - FINE";
}*/


char *visionTest(){
    if(total_robotDirection(DIRECTION_NORTH, DIRECTION_NORTH) != FORWARD){
        return "visionTest - ERROR: total_robotDirection(DIRECTION_NORTH, DIRECTION_NORTH) != FORWARD";
    }
    if(total_robotDirection(DIRECTION_NORTH, DIRECTION_EAST) != RIGHT){
        return "visionTest - ERROR: total_robotDirection(DIRECTION_NORTH, DIRECTION_EAST) != RIGHT";
    }
    if(total_robotDirection(DIRECTION_NORTH, DIRECTION_SOUTH) != BACKWARD){
        return "visionTest - ERROR: total_robotDirection(DIRECTION_NORTH, DIRECTION_SOUTH) != BACKWARD";
    }
    if(total_robotDirection(DIRECTION_NORTH, DIRECTION_WEST) != LEFT){
        return "visionTest - ERROR: total_robotDirection(DIRECTION_NORTH, DIRECTION_WEST) != LEFT";
    }

    if(total_robotDirection(DIRECTION_SOUTH, DIRECTION_NORTH) != BACKWARD){
        return "visionTest - ERROR: total_robotDirection(DIRECTION_SOUTH, DIRECTION_NORTH) != BACKWARD";
    }
    if(total_robotDirection(DIRECTION_SOUTH, DIRECTION_EAST) != LEFT){
        return "visionTest - ERROR: total_robotDirection(DIRECTION_SOUTH, DIRECTION_EAST) != LEFT";
    }
    if(total_robotDirection(DIRECTION_SOUTH, DIRECTION_SOUTH) != FORWARD){
        return "visionTest - ERROR: total_robotDirection(DIRECTION_SOUTH, DIRECTION_SOUTH) != FORWARD";
    }
    if(total_robotDirection(DIRECTION_SOUTH, DIRECTION_WEST) != RIGHT){
        return "visionTest - ERROR: total_robotDirection(DIRECTION_SOUTH, DIRECTION_WEST) != RIGHT";
    }

    if(total_robotDirection(DIRECTION_EAST, DIRECTION_NORTH) != LEFT){
        return "visionTest - ERROR: total_robotDirection(DIRECTION_EAST, DIRECTION_NORTH) != LEFT";
    }
    if(total_robotDirection(DIRECTION_EAST, DIRECTION_EAST) != FORWARD){
        return "visionTest - ERROR: total_robotDirection(DIRECTION_EAST, DIRECTION_EAST) != FORWARD";
    }
    if(total_robotDirection(DIRECTION_EAST, DIRECTION_SOUTH) != RIGHT){
        return "visionTest - ERROR: total_robotDirection(DIRECTION_EAST, DIRECTION_SOUTH) != RIGHT";
    }
    if(total_robotDirection(DIRECTION_EAST, DIRECTION_WEST) != BACKWARD){
        return "visionTest - ERROR: total_robotDirection(DIRECTION_EAST, DIRECTION_WEST) != BACKWARD";
    }

    if(total_robotDirection(DIRECTION_WEST, DIRECTION_NORTH) != RIGHT){
        return "visionTest - ERROR: total_robotDirection(DIRECTION_WEST, DIRECTION_NORTH) != RIGHT";
    }
    if(total_robotDirection(DIRECTION_WEST, DIRECTION_EAST) != BACKWARD){
        return "visionTest - ERROR: total_robotDirection(DIRECTION_WEST, DIRECTION_EAST) != BACKWARD";
    }
    if(total_robotDirection(DIRECTION_WEST, DIRECTION_SOUTH) != LEFT){
        return "visionTest - ERROR: total_robotDirection(DIRECTION_WEST, DIRECTION_SOUTH) != LEFT";
    }
    if(total_robotDirection(DIRECTION_WEST, DIRECTION_WEST) != FORWARD){
        return "visionTest - ERROR: total_robotDirection(DIRECTION_WEST, DIRECTION_WEST) != FORWARD";
    }

    return "visionTest - FINE";
}


void testAll(){
    communication_log(LEVEL_INFO, totalOrientationTest());
    //communication_log(LEVEL_INFO, isExitTest());
    communication_log(LEVEL_INFO, visionTest());
    communication_log(LEVEL_INFO, test_getTileCoordinates());
    communication_log(LEVEL_INFO, angleTest());
}