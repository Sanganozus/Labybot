#include "tasks.h"

#include <communication/communication.h>
#include <inttypes.h>
#include <stdlib.h>


//----- Task Class -----//


void startTask(Task* task){
    task->startMethod(task->startParameters);
}

Task* createTask(TaskMethod startMethod, CancelParameters* cancelParameters, Parameters* startParameters){
    Task* task = (Task*)malloc(sizeof(Task));
    task->startMethod = startMethod;
    task->cancelParameters = cancelParameters;
    task->startParameters = startParameters;
    return task;
}

Parameters* createParameters(Direction_t direction, uint16_t speed, float fixedValue) {
    Parameters* parameters = (Parameters*)malloc(sizeof(Parameters));
    parameters->direction = direction;
    parameters->speed = speed;
    parameters->fixedValue = fixedValue;
    return parameters;
}

CancelParameters* createCancelParameters(CancelType type, float abortPar_1, float abortPar_2){
    CancelParameters* parameters = (CancelParameters*)malloc(sizeof(CancelParameters));
    parameters->type = type;

    parameters->abortPar_1 = abortPar_1;
    parameters->abortPar_2 = abortPar_2;
    
    return parameters;
}