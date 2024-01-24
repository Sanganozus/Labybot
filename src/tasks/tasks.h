#ifndef TASK_H
#define TASK_H

#include <stdint.h>
#include <communication/packetTypes.h>

//******************//
/*
Aufgabe: 
Kümmert sich um die Aufgaben, die der Roboter ausführen soll
z.B. fahre vorwärts für 5 Sekunden oder dreh dich um 90 Grad


Bietet Funktionen für den Umgang mit den Tasks an:
 - startTask(): Startet übergebenen Task und kümmert sich um Einzelheiten, wie die Bedingungen, wann der Task abgebrochen werden soll (immer verwenden, wenn Task gestartet werden soll)
 - createTask(): Erstellt einen neuen Task aus übergebenen Parametern
 - createParameters: Erstellt einen Parameter-Struct (nötige Parameter für die Tasks) aus den übergebenen Parametern
*/
//******************//

/**
 * Struct für die Parameter, die zum Starten des Tasks benötigt werden
 * 
 * Beinhaltet:
 * - speed: Geschwindigkeit
 * - steering: Drehgeschwindigkeit
*/
typedef struct {
    Direction_t direction;
    uint16_t speed;
    float fixedValue;
} Parameters;

typedef void (*TaskMethod)(Parameters*);

typedef enum {
    NONE,
    DURATION,
    COORDINATES,
    ANGLE
} CancelType;

/**
 * Struct für die Parameter, die zum Abbruch des Tasks benötigt werden
 * 
 * Beinhaltet:
 * - type: Wie soll geprüft werden, wann der Task abgebrochen wird? (z.B. nach bestimmter Zeit, Distand oder Winkel?)
 * - abortPar_1: Zeiteinheit (in ms), Distanz (in mm) oder Winkel (in Grad)
*/
typedef struct {
    CancelType type;
    float abortPar_1;
    float abortPar_2;
} CancelParameters;

/**
 * Struct für die Aufgaben, die der Roboter ausführen soll
 * 
 * Beinhaltet:
 * - startMethod: Methode, die im Task ausgeführt werden soll
 * - abortMethod: Methode, die beim Abbruch ausgeführt werden soll
 * - cancelParameters: Parameter, die zum Abbruch des Tasks benötigt werden
 * - startParameters: Parameter, die zum Ausführen des Tasks benötigt werden
*/
typedef struct {
    TaskMethod startMethod;

    CancelParameters* cancelParameters;
    Parameters* startParameters;
} Task;

/**
 * Startet den übergebenen Task.
 * 
 * @param task auszuführenden Task
*/
void startTask(Task* task);

/**
 * Erstellt neuen Task aus übergebenen Parametern
 * 
 * @param startMethod Methode, die im Task ausgeführt werden soll
 * @param abortMethod Methode, die beim Abbrechen des Tasks ausgeführt werden soll
 * @param cancelParameters Parameter, die zum Abbruch des Tasks benötigt werden
 * @param startParameters Startparameter, die zur Ausführung des Tasks benötigt werden
*/
Task* createTask(TaskMethod startMethod, CancelParameters* cancelParameters, Parameters* startParameters);

/**
 * Erstellt einen Parameter-Struct aus den übergebenen Parametern
 * 
 * @param speed Geschwindigkeit
 * @param steering Drehgeschwindigkeit
*/
Parameters* createParameters(Direction_t direction, uint16_t speed, float fixedValue);

/**
 * Erstellt einen CancelParameter-Struct aus den übergebenen Parametern
 * 
 * @param type Art des Abbruchs
 * @param abortPar_1 Abbruchparameter
*/
CancelParameters* createCancelParameters(CancelType type, float abortPar_1, float abortPar_2);

#endif /* TASK_H */
