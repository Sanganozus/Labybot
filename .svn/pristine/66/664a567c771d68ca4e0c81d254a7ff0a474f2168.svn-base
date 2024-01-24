#ifndef POSE_H
#define POSE_H

#include "communication/packetTypes.h"
#include "stdint.h"


// robot's pose
extern float thetaTemp;
extern Pose_t pose;
extern Pose_t * pose_Temp;
bool checkAprilPose;

extern Pose_t *getPose();

void poseUpdate();

bool firstAprilTagUpdate();




/*
aktualisiere die Pose (aber nur, wenn gerade kein Task ausgeführt wird, um Inkosistenzen zu vermeiden)

Sollte immer nach dem Beenden eines Tasks ausgeführt werden, da Funktion auch schaut,
ob wieder eine neue Pose vom AprilTag gesendet wurde und die Pose falls ja entsprechend updatet.

*/
void aktualisierePose();


void poseUpdateAprilTag(const uint8_t* packet, __attribute__((unused)) const uint16_t size);

void requestAprilTagPose(GetPose_t * aprilTag);

void timeTask_RequestAprilTag();

Direction_t pose_getCurrentCardinalDirection();

/**
 * @param direction Cardinal Direction (NORTH, EAST, SOUTH, WEST)
 * 
 * @returns Winkel, in dem sich die Richtung befindet
 * NORTH: Pi/2
 * EAST: 0
 * SOUTH: -Pi/2
 * WEST: Pi
*/
float getAngle_forCardinalDirection(Direction_t direction);

//----- Für das Messen der Änderungen -----//

/**
 * Beginnt die Messung der Winkeländerung
 * Dabei wird thetaDiff auf 0 gesetzt und es wird neu gezählt
*/
void startMeasuring_thetaDiff();

/**
 * Gibt die Änderung des Winkels zurück seit dem letzten Aufruf von startMeasuring_thetaDiff()
*/
float getThetaDiff();

/**
 * Beginnt die Messung der Distanzänderung
 * Dabei wird distDiff auf 0 gesetzt und es wird neu gezählt
*/
void startMeasuring_distDiff();

/**
 * Gibt die Änderung des Winkels zurück seit dem letzten Aufruf von startDistDiff_measuring()
*/
float getDistDiff();

/**
 * 
 * @param direction Cardinal Direction (NORTH, EAST, SOUTH, WEST)
 * @param correctionValue Wert, um den die Pose korrigiert werden soll (nach rechts => positiv, nach links => negativ)
*/
void correctPose(Direction_t direction, float correctionValue);


#endif
