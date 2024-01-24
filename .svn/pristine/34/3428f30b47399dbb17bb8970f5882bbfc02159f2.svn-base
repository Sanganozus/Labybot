#ifndef VISION_H
#define VISION_H



#include <stdbool.h>
#include <stdint.h>
#include "communication/packetTypes.h"
#include "../explorer/robotDirection.h"

//******************//
/*
Aufgabe: 
Erkennung von Abständen, Wänden usw..
*/
//******************//

/**
 * Prüft, ob sich in der angegebenen Richtung eine Wand befindet
 * 
 * @param dir Richtung, für die geprüft werden soll (FORWARD, RIGHT, BACKWARD, LEFT)
 * 
 * @returns true, falls Wand vorhanden, sonst false. Bei Richtung BACKWARDS wird grundsätzlich false zurückgegeben!!!
*/
bool isWall(RobotDirection_t dir);

/**
 * Gibt, falls Wand vor dem Roboter vorhanden, die Entfernung zurück
 * 
 * @returns Entfernung zur Wand. Falls keine Wand vorhanden, gibt -1 zurück
*/
uint16_t getDistanceToWall(RobotDirection_t dir);

/**
 * Gibt den Abstand der momentanen Roboterposition zur realen Mitte zurück.
 * Dazu wird die Mitte ausgehend der Wanddistanzen links und rechts berechnet, so dass diese balanciert werdem.
 * 
 * Mehr nach rechts ausgleichen => positiv
 * Mehr nach links ausgleichen => negativ
 * 
 * Falls nicht isWall(LEFT) und isWall(RIGHT) true sind, wird -1 zurückgegeben
 * 
 * Zum Beispiel: 
 * - left: 13, right: 11 -> 1 nach rechts, damit der Abstand zum Mittelpunkt 12 (Mittelwert) beträgt
 * - left: 10, right: 13 -> -1.5 nach links, damit der Abstand zum Mittelpunkt 11,5 (Mittelwert) beträgt
*/
float poseCorrectionValue_withWalls();



#endif