#ifndef _TASK_MANAGEMENT_H
#define _TASK_MANAGEMENT_H

#include "taskqueue.h"
#include "taskMethods.h"

#include <communication/packetTypes.h>

#include <stdlib.h>
#include <stdbool.h>


//******************//
/*
Aufgabe: 
Schnittstelle für die Ausführung der Tasks, welche sich in der Taskqueue befinden.
Kümmert sich um:
- das Iterieren durch die Taskqueue
- Pause zwischen dem Ausführen der einzelnen Tasks
- das bedingte Abbrechen eines Tasks (nach einer Zeit, Distanz, Winkel, ...)

*/
//******************//

void setBreakTime(uint16_t breakTime);

void setPoseCorrection(bool on);

//----- Für die Probleme mit den April Tags folgende Methoden: -----//

bool isTaskActive();

void pauseTasks();

void unpauseTasks();

//-----//

/**
 * Gibt an, ob die Task Queue momentan iteriert wird
 * 
 * @returns 0: wird nicht iteriert, 1: wird iteriert
*/
bool isTaskQueueIterating();



/**
 * Beginnt die Ausführung der Tasks in der Queue
*/
void startQueue(Direction_t dir);

/**
 * Stoppt die Ausführung der Tasks in der Queue
 * 
 * ACHTUNG:
 * Momentaner Task wird abgebrochen und beim nächsten Anschalten befindet sich dieser nicht wieder in der Queue.
 * => Beim nächsten Anschalten wird der nachfolgende Task zuerst ausgeführt.
*/
void stopQueue();

/**
 * Fügt der Task Queue einen neuen Task hinzu
 * 
 * @param task Task, welcher hinzugefügt werden soll
*/
void addTask(TaskMethod startingMethod, CancelType cancelType, uint16_t abortPar_1, uint16_t abortPar_2, uint16_t speed, uint16_t steering);

void addTask_task(Task *task);
/**
 * Überspringt den momentanen Task in der Queue und führt den nächsten aus
*/
void skipTask();

/**
 * Überschreibt die Task Queue
 * 
 * @param queue 
*/
void setQueue(Queue* queue);

/**
 * Initialisiert die Task Queue auf die Test Queue
*/
void initTestQueue();

/**
 * Setzt die Task Queue zurück auf eine leere Queue
*/
void resetQueue();

/**
 * Entfernt den Task aus der Task Queue, welcher sich am kürzesten in der Queue befindet
*/
void removeLastTask();

/**
 * Gibt an, wie viele Tasks sich in der Task Queue befinden
 * 
 * @returns Task Queue Size
*/

uint16_t getTaskQueueSize();

/**
 * Lässt den Roboter in die gegebene Kardinalrichtung drehen
 * 
 * @param speed Drehgeschwindigkeit
 * @param dir Kardinalrichtung, in die gedreht werden soll
*/
void enqueue_rotateToCardinalDirection(uint16_t speed, Direction_t dir);

/**
 * Lässt den Roboter eine Zelle nach vorne fahren
 * 
 * @param speed Fahrgeschwindigkeit
 * @param dir Richtung, in die nach vorne gefahren soll
*/
void enqueue_moveForward_oneTile(uint16_t speed, Direction_t dir);


void manageTasks();

#endif
