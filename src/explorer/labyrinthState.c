#include "labyrinthState.h"

#include "tools/labyrinth/labyrinth.h"

#include <avr/pgmspace.h> 
#include <string.h>

uint16_t moves;
uint8_t visited[(LABYRINTH_COLS+1)*LABYRINTH_ROWS + (LABYRINTH_ROWS+1)*LABYRINTH_COLS];

void initLabyrinthState() {
    for(int i = 0; i < sizeof(visited); i++){
        visited[i] = 0;
    }
    moves = 0;
}

uint8_t labyrinth_getWidth() {
    return LABYRINTH_ROWS;
}

uint8_t labyrinth_getHeight() {
    return LABYRINTH_COLS;
}

uint16_t labyrinth_newMove() {
    return ++moves;
}

uint16_t labyrinth_getMoves() {
    return moves;
}

bool labyrinth_isWall(uint16_t row, uint16_t col, Direction_t dir) {
    switch(dir){
        case DIRECTION_NORTH:
            return labyrinth_getWalls(row, col).wall.north;
            break;
        case DIRECTION_EAST:
            return labyrinth_getWalls(row, col).wall.east;
            break;
        case DIRECTION_SOUTH:
            return labyrinth_getWalls(row, col).wall.south;
            break;
        case DIRECTION_WEST:
            return labyrinth_getWalls(row, col).wall.west;
            break;
        default:
            return -1;

    }
}

void labyrinth_setWall(uint16_t row, uint16_t col, Direction_t dir, bool isWall) {
    Walls_t walls = labyrinth_getWalls(row, col);
    switch(dir){
        case DIRECTION_NORTH:
            walls.wall.north = isWall;
            break;
        case DIRECTION_EAST:
            walls.wall.east = isWall;
            break;
        case DIRECTION_SOUTH:
            walls.wall.south = isWall;
            break;
        case DIRECTION_WEST:
            walls.wall.west = isWall;
            break;
    }

    labyrinth_setWalls(row, col, walls);
    communication_writePacket(CH_OUT_LABY_WALLS, (uint8_t*)&walls, sizeof(walls));
}

uint8_t checkVisited_inDirection(uint16_t row, uint16_t col, Direction_t dir) {
    uint16_t index = labyrinth_getWallIndex(row, col, dir);

    return visited[index];
}

void addVisited_inDirection(uint16_t row, uint16_t col, Direction_t dir) {
    uint16_t index = labyrinth_getWallIndex(row, col, dir);

    communication_log_P(LEVEL_INFO, PSTR("addVisited. vorher: visited[index]:%i"), visited[index]);

    communication_log_P(LEVEL_INFO, PSTR("addVisited. row:%i, col:%i, dir:%s, index:%i"), row, col, cardStr(dir), index);
    visited[index] = visited[index] + 1;
    communication_log_P(LEVEL_INFO, PSTR("addVisited. nachher: visited[index]:%i"), visited[index]);
}

void setVisited_inDirection(uint16_t row, uint16_t col, Direction_t dir, uint8_t val) {
    uint16_t index = labyrinth_getWallIndex(row, col, dir);

    visited[index] = val;
}

float getTile_x(uint16_t col) {
    return ((float)col - 4.0f) * LABY_CELLSIZE;
}

float getTile_y(uint16_t row) {
    return -((float)row - 4.0f) * LABY_CELLSIZE;
}