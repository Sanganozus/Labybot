#ifndef QUEUE_H
#define QUEUE_H

#include "tasks.h"

#include <stdbool.h>
#include <stdint.h>


//******************//
/*
Aufgabe: 
Kümmert sich um die Schlangenstruktur

Bietet Funktionen für den Umgang mit der Schlange an:
 - createQueue(): erstellt eine neue Schlange (immer verwenden, wenn neue Schlange benutztwerden soll!!!)
 - enqueue(): Fügt der Schlange neuen Task hinzu
 - dequeue(): Holt aktuellen Task aus der Schlange und entfernt diesen
 - isEmpty(): Gibt an, ob die Schlange leer ist
 - queueSize(): Gibt an, wie viele Tasks sich momentan in der Schlange befinden
*/
//******************//


/**
 * Struktur für einen Knoten in der Schlange
 * 
 * Besteht aus:
 * - task: Pointer auf momentanen Task
 * - next: Pointer auf nächsten Knoten
*/
/**

*/

typedef struct TaskNode {
    Task* task;
    struct TaskNode* next;
} TaskNode;

/**
 * Struktur für Task-Schlange
 * 
 * Besteht aus:
 * - front: Pointer auf Knoten des ältesten Tasks in der Schlange
 * - rear: Pointer auf Knoten des jüngsten Tasks in der Schlange
*/
typedef struct {
    TaskNode* front;
    TaskNode* rear;
} Queue;

// Funktionen zur Schlange

/**
 * Erstellt eine neue Schlange und gibt diese zurück
 * 
 * @return Neu erstellte leere Schlange
*/
Queue* createQueue();

/**
 * Fügt der Schlange einen neuen Task hinzu
 * 
 * @param queue Schlange, der der neue Task hinzugefügt werden soll
 * @param task Task, der hinzugefügt werden soll 
*/
void enqueue(Queue* queue, Task* task);

/**
 * Gibt nächsten Task zurück
 * 
 * @param queue zu verwendende Schlange
 * 
 * @returns Pointer auf nächsten Task
*/
Task* dequeue(Queue* queue);

/**
 * Gibt an, ob die Schlange leer ist
 * 
 * @param queue zu prüfende Schlange
 * 
 * @returns 0: Schlange nicht leer, 1: Schlange leer
*/
bool isEmpty(Queue* queue);

/**
 * Gibt an, wie viele Tasks sich momentan in der Schlange befinden
 * 
 * @param queue zu verwendende Schlange
 * 
 * @returns Anzahl der Tasks in der Schlange
*/
uint8_t queueSize(Queue* queue);


#endif /* QUEUE_H */
