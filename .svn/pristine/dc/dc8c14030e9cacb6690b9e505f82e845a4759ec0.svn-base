#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>

//******************//
/*
Aufgabe: 
Kümmert sich um die Sensordaten und dient als Interface zu den Sensordaten

Folgendes soll dieses Modul erfüllen:
- ermöglicht Das Auslesen der benötigten Sensordaten
- für die Encoderwerte: relative und absolute Counts (also wie oft insgesamt gedreht und wie oft seit dem letzten Reset gedreht (Wie beim Tacho im Auto die Kilometerzahl))
- Methoden für die Übersetzung von Distance und Angle in Time für die CancelConditions implementieren?

Bietet Funktionen für das Auslesen der Sensordaten an:
- ???

 Wie verwenden?
 - Am besten nicht auf ISRCustom und initSensors zugreifen, sondern alles über diese Schnittstelle machen
 - 
*/
//******************//

void resetCounts();



void getAndResetEncoders(int16_t* encoder1, int16_t* encoder2);

int16_t getEncoderVal1();

int16_t getEncoderVal2();

float getEncoder1MM();

float getEncoder2MM();

uint8_t getBumperCount();

void incrementBumped();

int getCount_leftWheel();

int getCount_rightWheel();

int getTotalCount_leftWheel();

int getTotalCount_rightWheel();


uint16_t getCounterBumper();
uint16_t getCounterBumperOld();
void setOldBumper(uint16_t val);

uint8_t getDirection1();
uint8_t getDirection2();

void setEncoder1MM(float val);
void setEncoder2MM(float val);

void initSensors();

/**
 * Enthält einen Timetask, welcher regelmäßig überprüft, ob eine Kollision aufgetreten ist
*/
void checkBumped();

/**
 * 
*/
void updateTelemetry();

#endif