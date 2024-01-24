#ifndef DRIVING_H
#define DRIVING_H

#include <communication/packetTypes.h>

//******************//
/*
Aufgabe: 
Kümmert sich um die Ansteuerung des Roboters. Bietet hierfür Schnittstellen an, welche sich um die Einzelheiten der Ansteuerung kümmern

Bietet folgende Funktionalitäten an:
- initDrive(): Gibt dem Roboter einen Fahrauftrag 
- initDrive_withBalancing_withPars(): Gibt dem Roboter einen Fahrauftrag, welcher Ungleichheiten in den Rädern ausgleichen soll (funktioniert bis jetzt nur beim Vorwärts- & Rückwärtsfahren)
- getSpeedA(): Gibt Momentangeschwindigkeit des linken Rads zurück
- getSpeedB(): Gibt Momentangeschwindigkeit des rechten Rads zurück
- stopDrive(): Lässt den Roboter anhalten
- stopBalancing(): Stoppt das Ausgleichen des Roboters 


 Wie verwenden?
 - 
 - 
*/
//******************//


/**
 * Lässt den Roboter fahren.
 * 
 * @param speed: Geschwindigkeit
 * @param steering: Drehgeschwindigkeit
*/
void initDrive(int speed, int steering);

void setToleranceValues(uint16_t tol_fixedValue, uint16_t tol_theta, uint16_t correctionValue);

/**
 * Lässt den Roboter fahren mit integriertem Ausgleichen von Ungleichheiten in den Rädern.
 * Kümmert sich um die Einzelheiten des Ausgleichens.
 * 
 * @param speed: Geschwindigkeit
 * @param steering: Drehgeschwindigkeit
 * @param direction: Kardinalrichtung, zu der hin ausgeglichen werden soll
 * @param fixedValue: x- bzw, y-Wert, zu dem hin ausgeglichen werden soll
*/
void initDrive_withBalancing_withPars(Direction_t direction, int speed, int steering, float fixedValue, float fixedTheta);

void initDrive_withBalancing(Direction_t currentDir, int speed, int steering);

void initDrive_withBalancing_withFixedValue(Direction_t currentDir, int speed, int steering, float fixedValue);

/**
 * @returns Geschwindigkeit des linken Rads
*/
int getSpeed_left();

/**
 * @returns Geschwindigkeit des rechten Rads
*/
int getSpeed_right();

/**
 * Lässt den Roboter anhalten
*/
void stopDrive();

/**
 * Kümmert sich darum, dass das Ausgleichen gestoppt wird. (Wichtig, da sonst Roboter nicht stoppt, auch wenn stopDrive() ausgeführt wurde)
*/
void stopBalancing();

/**
 * Kümmert sich um den TIMETASK, welcher das Balancing ausführt
*/
void checkBalancing();

#endif
