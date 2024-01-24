#ifndef ROBOT_H
#define LABYRINTH_H

#include <stdlib.h>
#include <tools/labyrinth/labyrinth.h>
#include <communication/packetTypes.h>

#include "../sensors/vision.h"


char *cardStr(Direction_t dir);

char *robStr(RobotDirection_t dir);

/**
 * Checks if the explorer can continue or the algorithm needs to be aborted.
 * this happens if the robot finds an exit, the maximum number of allowed moves is reached or a severe or user error ocurred.
 * The return value of this function must be tested inside your algorithm in an if or while statement.
 * 
 * @returns True: can continue, False: robot found exit or maximum number of moves is reached or error ocurred
*/
bool robot_canContinue();

/**
 * Determines if the robot escaped from the labyrinth
 * 
 * @returns True: robot escaped, False: robot didn't escape
*/
bool robot_hasEscaped();

/**
 * Signals that a User Error has ocurred
*/
void robot_userError();

/**
 * Checks if robot is able to move in the given direction (FORWARD, LEFT, BACKWARD, RIGHT)
 * 
 * @param robotDirection Direction to be tested
 * 
 * @returns True: robot can move in direction, False: robot can't move in direction
*/
bool robot_canMove(RobotDirection_t robotDirection);

/**
 * Checks if robot is able to move in the given cardinal direction (NORTH, EAST, SOUTH, WEST)
 * 
 * @param robotDirection Direction to be tested
 * 
 * @returns True: robot can move in direction, False: robot can't move in direction
*/
bool robot_canMoveCardinal(Direction_t direction);

RobotDirection_t total_robotDirection(Direction_t currentOrientation, Direction_t direction_toCheck);

//int8_t isExit(uint16_t col, uint16_t row, Direction_t currentDir, bool wall_forward, bool wall_right, bool wall_left);

/**
 * Checks if robot is adjacent to exit
 * 
 * @returns -1: if robot is not adjacent to exit, else the direction (FORWARD, LEFT, BACKWARD, RIGHT) to the exit
*/
int8_t robot_getExitDirection();

/**
 * Moves in the given direction
 * If direction not FORWARD or BACKWARDS, rotates in the given Direction
 * 
 * @param robotDirection Direction in which the robot should move (FORWARD, LEFT, BACKWARD, RIGHT)
 * 
 * @returns True: sucessfully executed, False: not executed
*/
bool robot_move(RobotDirection_t robotDirection);

/**
 * Rotates the robot in the given direction relative to the current orientation
 * 
 * @param robotDirection relative direction to be rotated to. (FORWARD, LEFT, BACKWARD, RIGHT)
*/
void robot_rotate(RobotDirection_t robotDirection);

/**
 * Rotates the robot in the given cardinal direction
 * 
 * @param robotDirection direction to be rotated to. (NORTH, EAST, SOUTH, WEST)
*/
void robot_rotateCardinal(Direction_t direction);

/**
 * @returns the current column of the robot (zero-based)
*/
uint16_t robot_getColumn();

/**
 * @returns the current row of the robot (zero-based)
*/
uint16_t robot_getRow();

/**
 * @returns current cardinal orientation of the robot (NORTH, EAST, SOUTH, WEST)
*/
Direction_t robot_getOrientation();

/**
 * calculates the cardinal direction, the robot is oriented to after rotating in the given direction
 * 
 * @param robotDirection relative direction to be rotated to. (FORWARD, LEFT, BACKWARD, RIGHT)
 * 
 * @returns the cardinal direction, the robot is oriented to after rotating in the given direction
*/
Direction_t robot_orientationAfterRotation(RobotDirection_t robotDirection);

Direction_t getTotalOrientation(RobotDirection_t robDir, Direction_t dir);

/**
 * @returns current Pose of the robot
*/
Pose_t *robot_getPose();

/**
 * @returns true: if all tasks in the queue have been executed, false: otherwise
*/
bool robot_tasksFinished();

/**
 * Führt die Tasks aus, die während des momentanen Moves in die Queue eingefügt wurden
*/
void start();

/**
 * Gibt den Korrekturwert für die Berechnung des Mittelpunktes des zu fahrenden Felds zurück
*/
float getCoordinates_correctionValue();

#endif 