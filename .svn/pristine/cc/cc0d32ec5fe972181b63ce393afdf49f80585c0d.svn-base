#ifndef MAIN_H
#define MAIN_H

#include <stdbool.h>
#include <stdint.h>


//Debug-Hilfsvariablen
extern bool logPose;
extern bool logTelemetry;
extern bool logQueue;
extern bool logExplorer;
extern bool logBalancing;
extern bool logDist;
extern bool logPoseCorrection;
extern bool logCoordinatesCorrection;

extern bool switch_poseCorrection;
extern bool switch_coordinatesCorrection;

extern bool debugContinue;

extern float korrekturLinkesRad;
extern float korrekturRechtesRad;

extern float achsenlaenge;
extern float correctionValue;
extern float tolerance_fixedValue;

/**
 * Gibt den Korrekturwert für die Ermittlung des Mittelpunkts des nächsten Felds nach Rotation in Richtung dir zurück
*/
float getCorrectionValue_rotate(uint8_t dir);

#endif /* MAIN_H */