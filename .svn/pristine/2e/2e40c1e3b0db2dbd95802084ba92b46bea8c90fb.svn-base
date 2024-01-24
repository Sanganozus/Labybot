#include "snake.h"

#include <stdlib.h>

// Funktion zum Erstellen einer leeren Schlange
Queue* createQueue() {
    Queue* queue = (Queue*)malloc(sizeof(Queue));
    queue->front = queue->rear = NULL;
    return queue;
}

// Funktion zum Hinzufügen eines Elements zur Schlange (enqueue)
void enqueue(Queue* queue, Task* task) {
    TaskNode* newNode = (TaskNode*)malloc(sizeof(TaskNode));
    newNode->task = task;
    newNode->next = NULL;
    if (queue->rear == NULL) {
        queue->front = queue->rear = newNode;
        return;
    }
    queue->rear->next = newNode;
    queue->rear = newNode;
}

// Funktion zum Entfernen eines Elements aus der Schlange (dequeue)
Task* dequeue(Queue* queue) {
    if (queue->front == NULL) {
        return NULL;
    }
    Task* task = queue->front->task;
    TaskNode* temp = queue->front;
    queue->front = queue->front->next;
    free(temp);

    if (queue->front == NULL) {
        queue->rear = NULL;
    }
    return task;
}

bool isEmpty(Queue* queue) {
    if(queue->rear == NULL){
        return 1;
    }
    return 0;
}

uint8_t queueSize(Queue* queue) {
    if(isEmpty(queue)){
        return 0;
    }
    uint8_t counter = 1;
    TaskNode* currentNode = queue->front;
    while(currentNode->next != NULL){
        counter = counter + 1;
        currentNode = currentNode->next;
    }
    return counter;
}