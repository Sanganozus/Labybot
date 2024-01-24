#ifndef LABYRINTH_STATE_H
#define LABYRINTH_STATE_H

#include <stdint.h>
#include <stdbool.h>

#include "communication/communication.h"


//******************//
/*
Aufgabe: 
Kümmert sich um die Instanz des Labyrinths und verwaltet Informationen darüber, wie oft ein Feld besucht wurde

*/
//******************//

void initLabyrinthState();

uint8_t labyrinth_getWidth();

uint8_t labyrinth_getHeight();

/**
 * Gibt zurück, wie viele Moves der Roboter bis jetzt gemacht hat und zählt nach oben
*/
uint16_t labyrinth_newMove();

/**
 * Gibt zurück, wie viele Moves der Roboter bis jetzt gemacht hat
*/
uint16_t labyrinth_getMoves();

bool labyrinth_isWall(uint16_t row, uint16_t col, Direction_t dir);

void labyrinth_setWall(uint16_t row, uint16_t col, Direction_t dir, bool isWall);

uint8_t checkVisited_inDirection(uint16_t row, uint16_t col, Direction_t dir);

void addVisited_inDirection(uint16_t row, uint16_t col, Direction_t dir);

void setVisited_inDirection(uint16_t row, uint16_t col, Direction_t dir, uint8_t val);

float getTile_x(uint16_t col);

float getTile_y(uint16_t row);


#endif /* LABYRINTH_STATE_H */