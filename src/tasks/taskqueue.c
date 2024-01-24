#include "taskqueue.h"

#include <stddef.h>
#include <communication/communication.h>
#include <avr/pgmspace.h>
#include <stdlib.h>

Queue* taskQueue = NULL;
Task* currentTask = NULL;

Queue* getTaskQueue(){
    if(taskQueue == NULL){
        communication_log_P(LEVEL_FINE, PSTR("taskqueue.c - getTaskQueue(): taskQueue null, will be created"));
        taskQueue = createQueue();
    }
    return taskQueue;
}

Task* getCurrentTask(){
    return currentTask;
}

bool hasCurrentTask() {
    return currentTask != NULL;
}

Task* nextTask(){

    if (currentTask != NULL) {
        free(currentTask->startParameters);
        free(currentTask);    
    }
    
    if(!isEmpty(getTaskQueue())){
        communication_log_P(LEVEL_FINE, PSTR("taskqueue.c - nextTask(): queue not empty, dequeueing next Task"));
        currentTask = dequeue(taskQueue);
        if(currentTask == NULL){
            communication_log_P(LEVEL_SEVERE, PSTR("taskqueue.c - nextTask(): queue not empty, but dequeued Task NULL"));
        }
        //free(currentTask->cancelParameters);
        return currentTask;
    }
    communication_log_P(LEVEL_FINE, PSTR("taskqueue.c - nextTask(): is empty"));
    return NULL;
}

void initCurrentTask(){
    if(currentTask != NULL){
        communication_log_P(LEVEL_FINE, PSTR("taskqueue.c - initCurrentTask(): startTask"));
        startTask(currentTask);
    } else {
        communication_log_P(LEVEL_SEVERE, PSTR("taskqueue.c - initCurrentTask(): currentTask NULL"));
    }
}

int isQueueEmpty(){
    if(taskQueue == NULL){
        taskQueue = createQueue();
    }
    return isEmpty(getTaskQueue());
}

void addTaskToQueue(Task* task){
    if(taskQueue == NULL){
        taskQueue = createQueue();
    }
    enqueue(taskQueue, task);
}

void removeLastTaskInQueue(){
    
}

void setTaskQueue(Queue* queue){
    free(taskQueue);
    taskQueue = queue;
}