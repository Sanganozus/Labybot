#include "robot.h"

#include <communication/communication.h>
#include "tools/labyrinth/labyrinth.h"
#include <math.h>
#include <avr/pgmspace.h> 

#include "../pose/pose.h"
#include "../tasks/taskManagement.h"
#include "labyrinthState.h"
#include "main.h"




uint8_t flagError = 0;

RobotDirection_t rotated_thisMove = FORWARD;

char *cardStr(Direction_t dir) {
	switch(dir){
		case DIRECTION_NORTH:
			return "North";
		case DIRECTION_EAST:
			return "East";
		case DIRECTION_SOUTH:
			return "South";
		case DIRECTION_WEST:
			return "West";
	}
	return "UNDEFINED";
}

char *robStr(RobotDirection_t dir) {
	switch(dir) {
		case FORWARD:
			return "Forward";
		case RIGHT:
			return "Right";
		case BACKWARD:
			return "Backward";
		case LEFT:
			return "Left";
	}
	return "UNDEFINED";
}

bool robot_canContinue(){ //TODO
    if (robot_hasEscaped() == false && flagError == 0 && labyrinth_getMoves() < LABYRINTH_COLS*LABYRINTH_ROWS*10) {
        return true;
    }
    return false;
}

bool robot_hasEscaped(){ //TODO
    if (robot_getColumn() > 7 || robot_getRow() > 7 || robot_getColumn() < 1 || robot_getRow() < 1) {
        return true;
    }
    return false;
}

void robot_userError(){ //TODO
    flagError = 1;
}

bool robot_canMove(RobotDirection_t robotDirection) {
    if(robotDirection != BACKWARD) {
        return !isWall(robotDirection);
    }
    //Pfusche checken
    return true;
}

bool robot_canMoveCardinal(Direction_t direction) {
    Direction_t currentDir = pose_getCurrentCardinalDirection();
    if(direction == currentDir){ //TODO: Funktioniert das?
        return robot_canMove(FORWARD);
    } else if(direction == (currentDir + 1) % 4){
        return robot_canMove(RIGHT);
    } else if(direction == (currentDir + 2) % 4){
        return robot_canMove(BACKWARD);
    } else if(direction == (currentDir + 3) % 4){   
        return robot_canMove(LEFT);
    }

    return false;
}

RobotDirection_t total_robotDirection(Direction_t currentOrientation, Direction_t direction_toCheck) {
    int diff = direction_toCheck - currentOrientation;
    switch(diff) {
        case 0:
            return FORWARD;
        case 1:
        case -3:
            return RIGHT;
        case 2:
        case -2:
            return BACKWARD;
        case -1:
        case 3:
            return LEFT;
    }
    return -1;
}

/*int8_t isExit(uint16_t col, uint16_t row, Direction_t currentDir, bool wall_forward, bool wall_right, bool wall_left){

    //linke äußere Wände checken
    if (col == 1) {
        if (currentDir != DIRECTION_EAST &&
            ((total_robotDirection(currentDir, DIRECTION_WEST) == FORWARD && wall_forward)
            || (total_robotDirection(currentDir, DIRECTION_WEST) == RIGHT && wall_right)
            || (total_robotDirection(currentDir, DIRECTION_WEST) == LEFT && wall_left))) return total_robotDirection(currentDir, DIRECTION_WEST);
    }
    //rechte äußere Wände checken
    if (col == 7) {
        if (currentDir != DIRECTION_WEST &&
            ((total_robotDirection(currentDir, DIRECTION_EAST) == FORWARD && wall_forward)
            || (total_robotDirection(currentDir, DIRECTION_EAST) == RIGHT && wall_right)
            || (total_robotDirection(currentDir, DIRECTION_EAST) == LEFT && wall_left))) return total_robotDirection(currentDir, DIRECTION_EAST);
    }
    //obere Wand (Norden)
    if (row == 1) {
        if (currentDir != DIRECTION_SOUTH &&
            ((total_robotDirection(currentDir, DIRECTION_NORTH) == FORWARD && wall_forward)
            || (total_robotDirection(currentDir, DIRECTION_NORTH) == RIGHT && wall_right)
            || (total_robotDirection(currentDir, DIRECTION_NORTH) == LEFT && wall_left))) return total_robotDirection(currentDir, DIRECTION_NORTH);
    }
    //untere Wand (Süden)
    if (row == 7) {
        if (currentDir != DIRECTION_NORTH &&
            ((total_robotDirection(currentDir, DIRECTION_SOUTH) == FORWARD && wall_forward)
            || (total_robotDirection(currentDir, DIRECTION_SOUTH) == RIGHT && wall_right)
            || (total_robotDirection(currentDir, DIRECTION_SOUTH) == LEFT && wall_left))) return total_robotDirection(currentDir, DIRECTION_SOUTH);
    }

    return -1;
}*/

int8_t robot_getExitDirection() { //TODO
if(logExplorer) communication_log_P(LEVEL_INFO, PSTR(""));
    Direction_t currentDir = robot_getOrientation();
    if(logExplorer) communication_log_P(LEVEL_INFO, PSTR("----- robot_getExitDirection(). currentDir: %s -----"), cardStr(currentDir));
    

    //linke äußere Wände checken
    if (robot_getColumn() == 1) {
        if (currentDir != DIRECTION_EAST && isWall(total_robotDirection(currentDir, DIRECTION_WEST)) == false) return total_robotDirection(currentDir, DIRECTION_WEST);
    }
    //rechte Äußere Wånde checken
    if (robot_getColumn() == 7) {
        if (currentDir != DIRECTION_WEST && isWall(total_robotDirection(currentDir, DIRECTION_EAST)) == false) return total_robotDirection(currentDir, DIRECTION_EAST);
    }
    //obere Wand (Norden)
    if (robot_getRow() == 1) {
        if (currentDir != DIRECTION_SOUTH && isWall(total_robotDirection(currentDir, DIRECTION_NORTH)) == false) return total_robotDirection(currentDir, DIRECTION_NORTH);
    }
    //untere Wand (Süden)
    if (robot_getRow() == 7) {
        if (currentDir != DIRECTION_NORTH && isWall(total_robotDirection(currentDir, DIRECTION_SOUTH)) == false) return total_robotDirection(currentDir, DIRECTION_SOUTH);
    }

    return -1; 
}

bool robot_move(RobotDirection_t robotDirection) {
    rotated_thisMove = robotDirection;

    if(robotDirection == BACKWARD) {
        robot_rotate(RIGHT);
        robot_rotate(BACKWARD);
    } else {
        robot_rotate(robotDirection);
    }

    if(!robot_canMove(robotDirection)) {
        return false;
    }

    enqueue_moveForward_oneTile(1500, robot_orientationAfterRotation(robotDirection));
    return true;
}

void robot_rotate(RobotDirection_t robotDirection) {
    enqueue_rotateToCardinalDirection(1500, robot_orientationAfterRotation(robotDirection));
}

void robot_rotateCardinal(Direction_t direction) {
    enqueue_rotateToCardinalDirection(1500, direction);
}

uint16_t robot_getColumn() {
    int16_t x = getPose()->x;
    uint16_t col = round((x) / LABY_CELLSIZE) + 4;
    return col;
}

uint16_t robot_getRow() {
    int16_t y = getPose()->y;
    uint16_t row = (-(int16_t)round((y) / LABY_CELLSIZE) + 4);
    return row;
}

Direction_t robot_getOrientation() {
    return pose_getCurrentCardinalDirection();
}

Direction_t robot_orientationAfterRotation(RobotDirection_t robotDirection) {
    Direction_t result = getTotalOrientation(robot_getOrientation(), robotDirection);
	return result;
}

Direction_t getTotalOrientation(RobotDirection_t robDir, Direction_t dir) {
    return (robDir + dir) % 4;
}

Pose_t *robot_getPose(){
    return getPose();
}

bool robot_tasksFinished(){
    return isQueueEmpty() && !isTaskActive();
}

bool hasRotated(){
    return rotated_thisMove;
}

void start(){
    if(switch_poseCorrection) setPoseCorrection(true);

    startQueue(robot_getOrientation());
}

float getCoordinates_correctionValue(){
    float correctionValue = getCorrectionValue_rotate(rotated_thisMove);
    if(logCoordinatesCorrection) communication_log_P(LEVEL_INFO, PSTR(""));
    if(logCoordinatesCorrection) communication_log_P(LEVEL_INFO, PSTR("----- getCoordinates_correctionValue. correctionValue: %.3f -----"), correctionValue);

    return correctionValue;
}