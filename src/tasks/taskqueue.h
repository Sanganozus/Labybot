#ifndef TASKQUEUE_H
#define TASKQUEUE_H

#include "snake.h"

//******************//
/*
Aufgabe: 
Kümmert sich um die Verwaltung der Schlange mit den auszuführenden Tasks.

Bietet Funktionen für den Umgang mit der Schlange an:
 - getTaskQueue(): Gibt die Schlange zurück, welche verwaltet wird
 - getCurrentTask(): Gibt den momentanen Task zurück
 - nextTask(): Springt in der Schlange zum nächsten Task
 - initCurrentTask(): Führt den momentanen Task aus
 - isQueueEmpty(): Gibt an, ob Schlange leer ist
 - addTaskToQueue(): Fügt der Schlange einen neuen Task hinzu

Wie verwenden?
- Im besten Falle die Funktionen getTaskQueue() und getCurrentTask() nicht verwenden, nur für die interne Verwendung (werden vielleicht noch vom Header entfernt???)
- wenn nächster Task ausgeführt werden soll: nextTask() um nächsten Task zu laden und dann initCurrentTask() zum Ausführen (evtl nach gewisser Pause (2 Sekunden) ausführen)
*/
//******************//

/**
 * Gibt die Schlange zurück, welche verwaltet wird.
 * 
 * @returns verwaltete Schlange
*/
Queue* getTaskQueue();

/**
 * Gibt den momentanen Task zurück.
 * 
 * @returns momentanen Task in der Schlange
*/
Task* getCurrentTask();

/**
 * @returns true: if currentTask != NULL, false: sonst
*/
bool hasCurrentTask();

/**
 * Springt in der Schlange zum nächsten Task.
 * 
 * @returns nächsten Task in der Schlange
*/
Task* nextTask();

/**
 * Führt den momentanen Task aus und kümmert sich um die Einzelheiten der Ausführung. (immer verwenden, nie Task händisch ausführen)
*/
void initCurrentTask();

/**
 * Gibt an, ob Schlange leer ist.
 * 
 * @returns 0: Schlange nicht leer, 1: Schlange leer
*/
int isQueueEmpty();

/**
 * Fügt der Schlange einen neuen Task hinzu.
 * 
 * @param task Pointer auf Task, welcher der Schlange hinzugefügt werden soll
*/
void addTaskToQueue(Task* task);

/**
 * Löscht den letzten Task
 * 
 * NOCH NICHT IMPLEMENTIERT !!! (Brauchen wir das?)
*/
void removeLastTaskInQueue();

/**
 * Überschreibt die Schlange.
 * Mit Vorsicht verwenden, momentane Schlange wird gelöscht !!!
 * 
 * @param queue Schlange, die zum Überschreiben verwendet werden soll
*/
void setTaskQueue(Queue* queue);

#endif