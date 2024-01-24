#include "robot.h"
#include "explorer.h"
#include "labyrinthState.h"
#include "main.h"

#include <avr/pgmspace.h>
#include <communication/communication.h>
#include <tools/timeTask/timeTask.h>
#include <tools/labyrinth/labyrinth.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>


/**
 *  Implement your labyrinth exploration strategy in the function explore().
 *
 *  Below an example implementation is given which uses a naive random strategy
 *  for exploring the labyrinth. There is no guarantee of finding an exit before
 *  the maximum number of allowed movements (10 times the number of labyrinth
 *  cells) is reached.
 *  
 *  The function explore() is called once by the simulator after the labyrinth
 *  has been created. It needs to perform the complete exploration until the
 *  the robot exits the labyrinth.
 *
 *  Make sure to use the function robot_canContinue() after every robot_move().
 *  If it returns false, the function explore() must return. This can happen
 *  under several conditions:
 *  - The robot has exited the labyrinth
 *  - The maximum number of allowed moves has been reached
 *  - A user error issued by robot_UserError() ocurred
 *
 *  You can use any function from the standard C libraries which are available
 *  for your C compiler. If necessary, include additional headers.
 *
 *  For displaying debug messages in the console within HWPCS, use the stream
 *  stderr instead of stdout. You have to put a newline \n at the end of the string.
 *  For example:
 *    fprintf(stderr, "My custom debug message: var=%.3f\n", var);
 *    fflush(stderr);
 */
 
typedef enum {
	PFAD,
	SACKGASSE,
	PLATZ
} TileType_t;
 
uint8_t min(uint8_t var1, uint8_t var2, uint8_t var3){
	uint8_t min = 3;
	if(var1 < min) {
		min = var1;
	}
	if(var2 < min) {
		min = var2;
	}
	if(var3 < min) {
		min = var3;
	}
	return min;
}

char *tileStr(TileType_t tyle){
	switch(tyle){
		case PFAD:
			return "PFAD";
		case SACKGASSE:
			return "SACKGASSE";
		case PLATZ:
			return "PLATZ";
		default:
			return "UNKNOWN_TILE";
	}
}


 
void moveInDirection(RobotDirection_t direction) {
	if(!robot_move(direction)) communication_log(LEVEL_SEVERE, "Robot can't move in direction: %s", robStr(direction));
}	

RobotDirection_t chooseRandomDirection(RobotDirection_t possibilities[3], uint8_t directions) {
	uint8_t random = (uint8_t)(rand() % directions);
	return possibilities[random];
}

/**
* Kümmert sich darum, dass z.B., wenn Feld 1 Nord besucht ist, auch Feld 2 Süd besucht wurde
*/
void updateVisited(Direction_t dir){
	uint8_t row = robot_getRow();
	uint8_t col = robot_getColumn();

	addVisited_inDirection(row, col, dir);
}

uint8_t checkVisited(RobotDirection_t dir) {
	return checkVisited_inDirection(robot_getRow(), robot_getColumn(), robot_orientationAfterRotation(dir));
}

void addVisited(RobotDirection_t direction){
	updateVisited(robot_orientationAfterRotation(direction));
}


TileType_t lastTile;
bool exploring = false;

void startExploring(void) {
	// Initialization
	communication_log_P(LEVEL_INFO, PSTR("Exploration started"));

	initLabyrinthState();
	lastTile = PLATZ;
	exploring = true;
	
}

bool isExploring() {
	return exploring;
}

void stopExploring(){
	exploring = false;
}

void mapWalls(){
	uint8_t row = robot_getRow();
	uint8_t col = robot_getColumn();
	
	if(robot_canMove(FORWARD)){
		labyrinth_setWall(row, col, robot_orientationAfterRotation(FORWARD), false);
	} else {
		labyrinth_setWall(row, col, robot_orientationAfterRotation(FORWARD), true);
	}
	if(robot_canMove(RIGHT)){
		labyrinth_setWall(row, col, robot_orientationAfterRotation(RIGHT), false);
	} else {
		labyrinth_setWall(row, col, robot_orientationAfterRotation(RIGHT), true);
	}
	if(robot_canMove(LEFT)){
		labyrinth_setWall(row, col, robot_orientationAfterRotation(LEFT), false);
	} else {
		labyrinth_setWall(row, col, robot_orientationAfterRotation(LEFT), true);
	}
}

void move(){
	RobotDirection_t possibilities[3];
	uint16_t move = labyrinth_newMove();

	communication_log_P(LEVEL_INFO, PSTR("Explorer:  Exploring - Move: %i"), move);
	if(logExplorer) communication_log_P(LEVEL_INFO, PSTR("Explorer:  Last Tile: %s"), tileStr(lastTile));

	// check if robot is located in the cell adjacent to the exit
	int8_t exitDir = robot_getExitDirection();
	if(exitDir != -1 && logExplorer) communication_log_P(LEVEL_INFO, PSTR("Explorer:  Exit Direction: %s"), robStr((RobotDirection_t)exitDir));

	//mapWalls();


	if (exitDir > -1) {
		// rotate in direction of exit and escape the labyrinth
		communication_log_P(LEVEL_INFO, PSTR("Explorer:  Found exit!"));

		robot_move((RobotDirection_t)exitDir);
	} else {
		// Number of directions in which we can move
		communication_log_P(LEVEL_INFO, PSTR("Explorer:  No exit found!"));

		uint8_t directions = 0;

		if (robot_canMove(LEFT))
			possibilities[directions++] = LEFT;

		if (robot_canMove(FORWARD))
			possibilities[directions++] = FORWARD;

		if (robot_canMove(RIGHT))
			possibilities[directions++] = RIGHT;
		
		RobotDirection_t direction;
		
		if(logExplorer) communication_log_P(LEVEL_INFO, PSTR("Directions: %i, canMove(FORWARD): %i, canMove(RIGHT): %i, canMove(LEFT): %i"), directions, robot_canMove(FORWARD), robot_canMove(RIGHT), robot_canMove(LEFT));
		
		// Turn around and go back if there are no other options
		if (directions == 0) { //Sackgasse
			communication_log_P(LEVEL_INFO, PSTR("Explorer:  Sackgasse"));

			if(!robot_move(BACKWARD)){
				communication_log(LEVEL_SEVERE, "Sackgasse: can't move in direction: %s", robStr(BACKWARD));
			}
			
			lastTile = SACKGASSE;
		} else { //Keine Sackgasse
			communication_log_P(LEVEL_INFO, PSTR("Explorer:  keine Sackgasse"));

			if (directions == 1){ //Pfad
				communication_log_P(LEVEL_INFO, PSTR("Explorer:  Pfad"));

				//Weg weiterverfolgen, bis ein Platz kommt

				direction = possibilities[0];
			
				lastTile = PFAD;
			} else { //Platz erreicht
				communication_log_P(LEVEL_INFO, PSTR("Explorer:  Platz"));

				uint8_t visitedForward = robot_canMove(FORWARD) ? checkVisited(FORWARD) : 3;
				uint8_t visitedRight = robot_canMove(RIGHT) ? checkVisited(RIGHT) : 3;
				uint8_t visitedBackward = robot_canMove(BACKWARD) ? checkVisited(BACKWARD) : 3;
				uint8_t visitedLeft = robot_canMove(LEFT) ? checkVisited(LEFT) : 3;
				
				if((robot_canMove(FORWARD) ? visitedForward : 0) 
				+ (robot_canMove(RIGHT) ? visitedRight : 0) 
				+ (robot_canMove(BACKWARD) ? visitedBackward : 0) 
				+ (robot_canMove(LEFT) ? visitedLeft : 0) 
				== 0){ //Platz noch unbekannt
					if(logExplorer) communication_log_P(LEVEL_INFO, PSTR("Explorer:  - unbekannter Platz"));
					if(logExplorer) communication_log_P(LEVEL_INFO, PSTR("Anzahl an möglichen Richtungen: %i"), directions);
					for(int i = 0; i < directions; i++){
						if(logExplorer) communication_log_P(LEVEL_INFO, PSTR("  - mögliche Richtung: %s"), robStr(possibilities[i]));
					}

					direction = chooseRandomDirection(possibilities, directions);
					if(logExplorer) communication_log_P(LEVEL_INFO, PSTR("  - gewählte direction: %s"), robStr(direction));
				} else { //Platz bekannt
					if(logExplorer) communication_log_P(LEVEL_INFO, PSTR("Explorer:  - bekannter Platz"));

					if(visitedBackward == 0){ //Weg zurück noch nicht abgelaufen
						if(logExplorer) communication_log_P(LEVEL_INFO, PSTR("Explorer:    -> Weg zurück noch nicht abgelaufen"));

						direction = BACKWARD;
					} else { //weg mit minimaler Anzahl an Markierungen
						if(logExplorer) communication_log_P(LEVEL_INFO, PSTR("Explorer:    -> Weg mit minimaler Anzahl an Markierungen ablaufen"));

						uint8_t minDirections = 0;
						RobotDirection_t minPossibilites[3];

						if(logExplorer) communication_log_P(LEVEL_INFO, PSTR("Explorer:      -> visitedForward: %i, visitedRight: %i, visitedLeft: %i"), visitedForward, visitedRight, visitedLeft);
						
						uint8_t minMarks = min(visitedForward, visitedRight, visitedLeft);
						
						if(visitedForward == minMarks && robot_canMove(FORWARD)){
							minPossibilites[minDirections++] = FORWARD;
						}
						if(visitedRight == minMarks && robot_canMove(RIGHT)){
							minPossibilites[minDirections++] = RIGHT;
						}
						if(visitedLeft == minMarks && robot_canMove(LEFT)){
							minPossibilites[minDirections++] = LEFT;
						}
						
						direction = chooseRandomDirection(minPossibilites, minDirections);

						if(logExplorer) communication_log_P(LEVEL_INFO, PSTR("Explorer:      -> chosen direction: %i"), (int)direction);
					}
				}
				if(lastTile != PLATZ){
					//Rückweg bekannt
					if(logExplorer) communication_log_P(LEVEL_INFO, PSTR("Explorer:  - lastTile != PLATZ => markiere Rückweg als bekannt"));
					addVisited(BACKWARD);
				}
				//Neuer Weg bekannt
				addVisited(direction);
					
				lastTile = PLATZ;
			}
			communication_log_P(LEVEL_INFO, PSTR("Explorer:  moving in direction: %s"), robStr(direction));
			moveInDirection(direction);
			move++;
		}
	}
	communication_log_P(LEVEL_INFO, PSTR("Explorer:  executing Tasks in Queue"));
	start();
}

bool hasRotatedForward = 0;

void explore(){
	TIMETASK(EXPLORE_TASK, 500){
		// robot_canContinue() needs to be called after every robot_move()
		// in order to ensure breaking the loop when the robot exits the labyrinth,
		// the maximum number of allowed moves is reached or some error occured
		//if(exploring && (debugContinue || !logExplorer)){
		if(exploring){
			if(robot_tasksFinished()){
				if(!hasRotatedForward){
					if(logExplorer) communication_log_P(LEVEL_INFO, PSTR("Explorer:  tasksFinished == TRUE && hasRotatedForward == FALSE"));
					robot_rotate(FORWARD);
					start();
					hasRotatedForward = 1;
				} else {
					hasRotatedForward = 0;
					//if(logExplorer) communication_log_P(LEVEL_INFO, PSTR(Explorer:  tasksFinished == TRUE");

					if(robot_canContinue()){ // Falls der Roboter nach jedem Move warten soll, bis manuell weitergemacht werden soll
						if(logExplorer) communication_log_P(LEVEL_INFO, PSTR("Explorer:  canContinue == TRUE"));
						if(logExplorer) debugContinue = 0;

						if(logExplorer) communication_log_P(LEVEL_INFO, PSTR(""));
						if(logExplorer) communication_log_P(LEVEL_INFO, PSTR(""));
						move();
					} else if(!robot_canContinue()){ 
						if(logExplorer) communication_log_P(LEVEL_INFO, PSTR("Explorer:  canContinue == FALSE"));

						exploring = false;
						communication_log_P(LEVEL_INFO, PSTR("Exploration finished"));
					}
				}
			} else {
				//if(logExplorer) communication_log_P(LEVEL_INFO, PSTR(Explorer:  tasksFinished == FALSE");
			}
		}
	}
}