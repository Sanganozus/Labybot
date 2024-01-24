#include "taskManagement.h"
#include "../pose/pose.h"
#include "../explorer/robot.h"
#include "../explorer/labyrinthState.h"
#include "../main.h"
#include "../driving/driving.h"
#include "../helper/mathHelper.h"

#include <stdlib.h>
#include <math.h>
#include <avr/pgmspace.h>
#include <tools/timeTask/timeTask.h>
#include <communication/communication.h>


//----- Hilfsvariablen -----//

//Für das Iterieren der Queue
static bool queue_iterating = 0;

static bool skip = 0;
static bool taskDone = 1;
static bool taskStopped = 0;

static bool poseCorrection = 0;
static uint8_t poseCorrectionCounter = 0;

static Direction_t currentDir = DIRECTION_NORTH;

//Für Timed Tasks
static uint16_t timedTask_time = 0;
static uint8_t timedTask_flag = 0; //0 (DEFAULT): ausgeschalten, 1: eingeschalten, 2: counting
static uint16_t timedTask_counter = 0;

//Für Angle Tasks
static float angleTask_startAngle = 0;
static uint8_t angleTask_flag = 0;
static float angleTask_abortAngle = 0;

//Für Distance Tasks
static uint8_t distanceTask_flag = 0;
static float distanceTask_startX = 0;
static float distanceTask_startY = 0;
static uint16_t distanceTask_distanceValue = 0;

//Für die Pause zwischen den Tasks
static uint8_t timerBeforeNextTask_flag = 1;  //0: ausgeschalten / nicht abgelaufen, 1 (DEFAULT): abgelaufen, 2: counting
static uint16_t timerBeforeNextTask_counter = 0;
static uint16_t timerBeforeNextTask_time = 2000;


//----- Debug- & Hilfsmethoden -----//

Queue *testQueue() {
    Queue *queue = createQueue();

    /*Parameters* testTask1_startParameters = createParameters(3000, 0);
    CancelParameters* testTask1_cancelParameters = createCancelParameters(DURATION, 3000);
    Task* testTask1 = createTask(getMethod_driveForward(), testTask1_cancelParameters, testTask1_startParameters); //Driving Forward

    Parameters* testTask2_startParameters = createParameters(3000, 0);
    CancelParameters* testTask2_cancelParameters = createCancelParameters(DURATION, 3000);
    Task* testTask2 = createTask(getMethod_driveBackwards(), testTask2_cancelParameters, testTask2_startParameters); //Driving Backwards

    Parameters* testTask3_startParameters = createParameters(3000, 0);
    CancelParameters* testTask3_cancelParameters = createCancelParameters(DURATION, 5000);
    Task* testTask3 = createTask(getMethod_rotate_clockwise(), testTask3_cancelParameters, testTask3_startParameters); //Rotating Clockwise

    enqueue(queue, testTask1);
    enqueue(queue, testTask2);
    enqueue(queue, testTask3);*/

    return queue;
}

void setBreakTime(uint16_t breakTime){
    timerBeforeNextTask_time = breakTime;
}

void setPoseCorrection(bool on){
    poseCorrection = on;
}



void logVariables(){
    communication_log_P(LEVEL_INFO, PSTR("  ->queue_iterating: %i, skip: %i, taskDone: %i, taskStopped: %i, timedTask_time: %i, timedTask_flag: %i, timedTask_counter: %i, angleTask_startAngle: %i, angleTask_flag: %i, angleTask_abortAngle: %i"), queue_iterating, skip, taskDone, taskStopped, timedTask_time, timedTask_flag, timedTask_counter, angleTask_startAngle, angleTask_flag, angleTask_abortAngle);
}


//----- Internal Methods -----//
void init_durationTask(uint16_t duration){
    if(logQueue) communication_log_P(LEVEL_INFO, PSTR(""));
    if(logQueue) communication_log_P(LEVEL_INFO, PSTR("----- init_durationTask. duration:%i -----"));
    timedTask_time = duration;

    timedTask_flag = 1;
}

void init_distanceTask(uint16_t distance){
    if(logQueue) communication_log_P(LEVEL_INFO, PSTR(""));
    if(logQueue) communication_log_P(LEVEL_INFO, PSTR("----- init_distanceTask. distance:%i -----"), distance);

    distanceTask_distanceValue = distance;

    distanceTask_flag = 1;
}

void init_angleTask(float angle){
    if(logQueue) communication_log_P(LEVEL_INFO, PSTR(""));
    if(logQueue) communication_log_P(LEVEL_INFO, PSTR("----- init_angleTask. angle:%.3f -----"), angle);

    float deltaAngle = angle_subtract(angle, getPose()->theta);

    if(deltaAngle >= -M_PI && deltaAngle <= 0.0f){
        getCurrentTask()->startMethod = getMethod_rotate_clockwise();
    } else if(deltaAngle > 0.0f && deltaAngle <= M_PI){
        getCurrentTask()->startMethod = getMethod_rotate_counterClockwise();
    } else {
        communication_log(LEVEL_SEVERE, "!!! Falsches Argument als Winkel übergeben !!!");
    }

    angleTask_abortAngle = deltaAngle;

    angleTask_flag = 1;
}

/**
 * Führt den nächsten Task in der Task Queue aus.
 * Integriert außerdem eine Pause von Default 2 Sekunden zum nächsten Task
*/
void startNextTask() {
    if(logQueue) communication_log_P(LEVEL_INFO, PSTR(""));
    if(logQueue) communication_log_P(LEVEL_INFO, PSTR("----- startNextTask -----"));

    Task* currentTask = nextTask();

    float coordinates_distance = 0.0f;
    float coordinates_correctionValue = 0.0f;

    switch(currentTask->cancelParameters->type){
        case DURATION:
            if(logQueue) communication_log_P(LEVEL_INFO, PSTR("  ->TaskManagement:  startNextTask -> Setting Abort by Duration. Duration: %i"), (int)currentTask->cancelParameters->abortPar_1);
            if(logQueue) communication_log_P(LEVEL_INFO, PSTR("  ->TaskManagement:  startNextTask -> Setting Abort by Duration. Duration als uint: %i"), (uint16_t)currentTask->cancelParameters->abortPar_1);
            if(logQueue) communication_log_P(LEVEL_INFO, PSTR("  ->TaskManagement:  startNextTask -> Setting Abort by Duration. Duration mit round als uint: %i"), (uint16_t)round(currentTask->cancelParameters->abortPar_1));
            init_durationTask(round(currentTask->cancelParameters->abortPar_1));
            break;
        case COORDINATES:
            coordinates_distance = getDistance(getPose()->x, getPose()->y, currentTask->cancelParameters->abortPar_1, currentTask->cancelParameters->abortPar_2);
            coordinates_correctionValue = getCoordinates_correctionValue();
            if(logQueue) communication_log_P(LEVEL_INFO, PSTR("  ->TaskManagement:  startNextTask -> Setting Abort by Distance. x-Wert: %.3f, y-Wert: %.3f, Distance: %i"), currentTask->cancelParameters->abortPar_1, currentTask->cancelParameters->abortPar_2, coordinates_distance);
            
            if(switch_coordinatesCorrection)  {
                coordinates_distance = coordinates_distance - coordinates_correctionValue;
                if(logQueue) communication_log_P(LEVEL_INFO, PSTR("    -> Correction: %.3f, Distance mit Correction: %.3f"), coordinates_correctionValue, coordinates_distance);
            }
            init_distanceTask(coordinates_distance);
            break;
        case ANGLE:
            if(logQueue) communication_log_P(LEVEL_INFO, PSTR("  ->TaskManagement:  startNextTask -> Setting Abort by Angle. Angle: %i"), (int)(currentTask->cancelParameters->abortPar_1*100));
            init_angleTask(currentTask->cancelParameters->abortPar_1);
            break;
        default:
            if(logQueue) communication_log(LEVEL_SEVERE, "TaskManagement:   startNextTask -> Unknown Cancel Type!");
            break;
    }

    timerBeforeNextTask_flag = 0;
    taskDone = 0;
    //free(currentTask);
}



void stopCurrentTask() {
    if(logQueue) communication_log_P(LEVEL_INFO, PSTR(""));
    if(logQueue) communication_log_P(LEVEL_INFO, PSTR("  -- stopCurrentTask --"));
    
    if(hasCurrentTask()){
        stopDrive();
    }

    timedTask_time = 0;
    timedTask_flag = 0;
    timedTask_counter = 0;

    angleTask_startAngle = 0;
    angleTask_flag = 0;
    angleTask_abortAngle = 0;

    distanceTask_flag = 0;
    distanceTask_startX = 0;
    distanceTask_startY = 0;
    distanceTask_distanceValue = 0;

    timerBeforeNextTask_flag = 1;
    timerBeforeNextTask_counter = 0;
}


//----- External Methods -----//
static bool pause_forAprilTag = 0;

bool isTaskActive(){
    return queue_iterating && (timedTask_flag == 2 || angleTask_flag == 2 || distanceTask_flag == 2);
}

void pauseTasks(){
    communication_log_P(LEVEL_INFO, PSTR("  -- pauseTasks --"));
    if(queue_iterating){
        communication_log_P(LEVEL_INFO, PSTR("  ->queue_iterating wird auf 0 gesetzt"));
        pause_forAprilTag = 1;
        queue_iterating = 0;
    }
}

void unpauseTasks(){
    communication_log_P(LEVEL_INFO, PSTR("  -- unpauseTasks --"));
    if(pause_forAprilTag) {
        communication_log_P(LEVEL_INFO, PSTR("  ->queue_iterating wird wieder auf 1 gesetzt"));
        queue_iterating = 1;
    }
}

bool isTaskQueueIterating() {
    return queue_iterating;
}

void startQueue(Direction_t dir) {
    if(logQueue) communication_log_P(LEVEL_INFO, PSTR(""));
    communication_log_P(LEVEL_INFO, PSTR("----- startQueue. dir:%s -----"), cardStr(dir));
    queue_iterating = 1;
    currentDir = dir;
}

void stopQueue() {
    if(logQueue) communication_log_P(LEVEL_INFO, PSTR("  -- stopQueue --"));
    if(logQueue) logVariables();

    stopCurrentTask();
    queue_iterating = 0;
    taskDone = 1;
}

void addTask(TaskMethod startingMethod, CancelType cancelType, uint16_t abortPar_1, uint16_t abortPar_2, uint16_t speed, uint16_t steering){
    Parameters* parameters = createParameters(pose_getCurrentCardinalDirection(), speed, steering);
    CancelParameters* cancelParameters = createCancelParameters(cancelType, abortPar_1, abortPar_2);
    Task* task = createTask(startingMethod, cancelParameters, parameters);
    addTask_task(task);
}

void addTask_task(Task* task){
    if(logQueue) communication_log_P(LEVEL_INFO, PSTR(""));
    if(logQueue) communication_log_P(LEVEL_INFO, PSTR("----- addTask_task -----"));
    addTaskToQueue(task);
    if(logQueue) communication_log(LEVEL_FINE, "  ->Task hinzugefügt. Queue size: %i", getTaskQueueSize());
}

void skipTask(){
    if(queue_iterating == 1){
        skip = 1;
    }
}

void setQueue(Queue* queue){
    setTaskQueue(queue);
}

void initTestQueue(){
    setQueue(testQueue());
}

void resetQueue(){
    setTaskQueue(createQueue());
}

void removeLastTask(){
    removeLastTaskInQueue();
}

uint16_t getTaskQueueSize(){
    return queueSize(getTaskQueue());
}

uint16_t queueIterationDebugCounter = 0;

void check_queueIteration() {
    TIMETASK(QUEUE_ITERATION_TASK, 150) {
        if(queue_iterating == 1){ //Schlange soll iteriert werden
            queueIterationDebugCounter++;
            if(skip == 1 || taskDone == 1) { //Prüfen, ob Task gewechselt werden soll (entweder manuell oder cancelCondition)

                if(!isQueueEmpty()) { //Queue hat neue Tasks
                    communication_log_P(LEVEL_INFO, PSTR("  ->TaskManagement:  Nächster Task wird bearbeitet"));

                    stopCurrentTask();
                    startNextTask();

                    taskStopped = 0;
                } else { //Keine neuen Tasks verfügbar
                    if(queueIterationDebugCounter == 10){
                        queueIterationDebugCounter = 0;
                    }
                    if(!taskStopped) {
                        communication_log_P(LEVEL_INFO, PSTR("  ->TaskManagement:  Keine neuen Tasks verfügbar"));
                        stopCurrentTask();
                        taskStopped = 1;
                    }
                }
            }
        }
    }
}



uint16_t debugCounter = 0;

void check_conditionalAbort(){
    TIMETASK(TIMED_TASK, 2) {

        //----- Abort by Duration -----//
        if(timedTask_flag == 2) {
            timedTask_counter = timedTask_counter + 1;
        }
        if(timedTask_counter % 1000 == 0 && timedTask_counter != 0){
            if(logQueue){
                communication_log_P(LEVEL_INFO, PSTR("  ->%i seconds passed"), timedTask_counter / 100);
            }
        }
        if (timedTask_flag == 1) {
            //Beim ersten Mal einschalten counter auf 0 zurücksetzen (Ist dies auch möglich, wenn nur im else if?)
            if(logQueue){
                communication_log_P(LEVEL_INFO, PSTR("  ->timedTask Flag auf 1, Timer wird gestellt"));
            }
            timedTask_counter = 0;
            timedTask_flag = 2;
        } else if (timedTask_flag == 2 && timedTask_counter == (timedTask_time / 10)) {
            //if(timedTask != NULL){ //<- funktioniert nicht (deshalb parameter timedTaskNull eingeführt)
            if(logQueue){
                communication_log_P(LEVEL_INFO, PSTR("  ->Timed Task vollendet"));
            }
            stopDrive();

            timedTask_counter = 0;
            timedTask_flag = 0;
            taskDone = 1;
        }


        
        //----- Abort by Angle -----//
        if(angleTask_flag == 1){
            angleTask_startAngle = getPose()->theta;
            angleTask_flag = 2;
            startMeasuring_thetaDiff();

            debugCounter = 0;
        }
        if(angleTask_flag == 2){
            if(logQueue) debugCounter++;

            if(debugCounter == 1000){
                debugCounter = 0;
                if(logQueue) communication_log_P(LEVEL_INFO, PSTR("  ->TaskManagement  Startwinkel: %i, Zielwinkel: %i, Momentanwinkel: %i, Winkeldiff: %.3f"), (int)(angleTask_startAngle*100), (int)(angleTask_abortAngle*100), (int)(getPose()->theta*100), fabs(angle_subtract(getPose()->theta, angleTask_startAngle)));
            }

            //Er soll sich um 3,13 drehen 
            //thetaDiff mehr als 3,13, 3,15 -> -3,13
            if(fabs(angle_subtract(getPose()->theta, angleTask_startAngle)) > fabs(angleTask_abortAngle) -0.01f){
                if(logQueue) communication_log_P(LEVEL_INFO, PSTR("  ->TaskManagement:  Angle Task vollendet. Differenz-geplant: %i, Differenz-real: %.3f"), (int)(angleTask_abortAngle*100), fabs(angle_subtract(getPose()->theta, angleTask_startAngle)));

                angleTask_flag = 0;

                stopDrive();
                taskDone = 1;
            }
        }


        //----- Abort by Distance -----//
        if(distanceTask_flag == 1){
            distanceTask_startX = getPose()->x;
            distanceTask_startY = getPose()->y;
            distanceTask_flag = 2;
            startMeasuring_distDiff();
        }
        if(distanceTask_flag == 2) {
            //Für die Pose-Korrektur
            poseCorrectionCounter++;
            if(poseCorrection && poseCorrectionValue_withWalls() != -1 && switch_poseCorrection && poseCorrectionCounter >= 100) {
                poseCorrectionCounter = 0;
                
                float poseCorrectionValue = 0.0f;
                switch(currentDir){
                    case DIRECTION_NORTH:
                    case DIRECTION_SOUTH:
                        poseCorrectionValue = getTile_x(robot_getColumn()) - poseCorrectionValue_withWalls();
                        break;
                    case DIRECTION_EAST:
                    case DIRECTION_WEST:
                        poseCorrectionValue = getTile_y(robot_getRow()) - poseCorrectionValue_withWalls();
                        break;
                }
                
                if(logPoseCorrection) communication_log_P(LEVEL_INFO, PSTR("  -> !! Pose-Korrektur. currentDir: %s, poseCorrectionValue: %.3f !!"), cardStr(currentDir), poseCorrectionValue);
                correctPose(currentDir, poseCorrectionValue);
            }

            if(logQueue) debugCounter++;

            if(debugCounter == 1000){
                debugCounter = 0;
                if(logQueue) communication_log_P(LEVEL_INFO, PSTR("  ->TaskManagement  Zieldistanz: %i, Momentandistanz: %i, DistDiff: %i"), distanceTask_distanceValue, (int)getDistance(distanceTask_startX, distanceTask_startY, getPose()->x, getPose()->y), abs((int)getDistDiff()));
            }

            if(abs((int)getDistance(distanceTask_startX, distanceTask_startY, getPose()->x, getPose()->y)) >= distanceTask_distanceValue) {
                if(logQueue) communication_log_P(LEVEL_INFO, PSTR("  ->TaskManagement:  Distance Task vollendet. Differenz-geplant: %i, Differenz-real: %i"), (int)distanceTask_distanceValue, (int)getDistance(distanceTask_startX, distanceTask_startY, getPose()->x, getPose()->y));

                distanceTask_flag = 0;

                stopDrive();

                taskDone = 1;

            }
        }
    }
}

void check_breakBetweenTasks(){
    TIMETASK(TIMERBEFORENEXTTASK_TASK, 100){
        if(timerBeforeNextTask_flag == 0){
            timerBeforeNextTask_counter = 0;
            timerBeforeNextTask_flag = 2;
        }
        if(timerBeforeNextTask_flag == 2){
            timerBeforeNextTask_counter++;
            if(timerBeforeNextTask_counter == (timerBeforeNextTask_time / 100)){
                timerBeforeNextTask_flag = 1;
                initCurrentTask();
            }
        }
    }
}

void manageTasks() {
    check_queueIteration();
    check_breakBetweenTasks();
    check_conditionalAbort();
}


Task *getTask_rotateToAngle(Direction_t dir, uint16_t speed, float angle){
    if(logQueue) communication_log_P(LEVEL_INFO, PSTR(""));
    if(logQueue) communication_log_P(LEVEL_INFO, PSTR("----- getTask_rotateToAngle. speed: %i, angle: %i -----"), speed, (int)(angle*100));
    TaskMethod method = getMethod_rotate_clockwise();

    /*if(angle >= -M_PI && angle <= 0.0f){
        method = getMethod_rotate_clockwise();
    } else if(angle > 0.0f && angle <= M_PI){
        method = getMethod_rotate_counterClockwise();
    } else {
        communication_log(LEVEL_SEVERE, "!!! Falsches Argument als Winkel übergeben !!!");
    }*/

    Parameters *pars = createParameters(dir, 1500, 0);
    CancelParameters *cancelPars = createCancelParameters(ANGLE, angle, 0.0f);

    return createTask(method, cancelPars, pars);
}

Task *getTask_moveForward_oneTile(Direction_t dir, uint16_t speed, float x, float y, float fixedValue) {
    if(logQueue) communication_log_P(LEVEL_INFO, PSTR(""));
    if(logQueue) communication_log_P(LEVEL_INFO, PSTR("----- getTask_moveForward_oneTile. speed:%i, x:%f, y:%f, fixedValue:%f -----"), speed, x, y, fixedValue);
    
    TaskMethod method = getMethod_driveForward_withFixedValue();

    Parameters *pars = createParameters(dir, speed, fixedValue);
    CancelParameters *cancelPars = createCancelParameters(COORDINATES, x, y);

    return createTask(method, cancelPars, pars);
}

//Wenn der Roboter rotiert wurde, -30 zur Distanz, ansonsten +30
//Nur bei Backward? Auch bei Left, Right???


void enqueue_rotateToCardinalDirection(uint16_t speed, Direction_t dir){
    if(logQueue) communication_log_P(LEVEL_INFO, PSTR(""));
    if(logQueue) communication_log_P(LEVEL_INFO, PSTR("----- enqueue_rotateToCardinalDirection. speed:%i, dir:%s -----"), speed, cardStr(dir));

    float currentAngle = getPose()->theta;
    float targetAngle = getAngle_forCardinalDirection(dir);

    if(logQueue) communication_log_P(LEVEL_INFO, PSTR("currentAngle: %.3f, targetAngle: %.3f"), currentAngle, targetAngle);

    addTask_task(getTask_rotateToAngle(dir, speed, targetAngle));
    
}

void enqueue_moveForward_oneTile(uint16_t speed, Direction_t dir){
    if(logQueue) communication_log_P(LEVEL_INFO, PSTR(""));
    if(logQueue) communication_log_P(LEVEL_INFO, PSTR("----- enqueue_moveForward_oneTile. speed:%i, dir:%s -----"), speed, cardStr(dir));

    float fixedValue = 0.0f;

    float tile_x = 0.0f;
    float tile_y = 0.0f;

    if(logQueue) communication_log_P(LEVEL_INFO, PSTR("  -> column: %i, row: %i"), robot_getColumn(), robot_getRow());

    switch(dir){
        case DIRECTION_NORTH:
            fixedValue = getTile_x(robot_getColumn());

            tile_x = getPose()->x;
            tile_y = getTile_y(robot_getRow() - 1);
            break;
        case DIRECTION_EAST:
            fixedValue = getTile_y(robot_getRow());

            tile_x = getTile_x(robot_getColumn() + 1);
            tile_y = getPose()->y;
            break;
        case DIRECTION_SOUTH:
            fixedValue = getTile_x(robot_getColumn());

            tile_x = getPose()->x;
            tile_y = getTile_y(robot_getRow() + 1);
            break;
        case DIRECTION_WEST:
            fixedValue = getTile_y(robot_getRow());

            tile_x = getTile_x(robot_getColumn() - 1);
            tile_y = getPose()->y;
            break;
    }
    if(logQueue) communication_log_P(LEVEL_INFO, PSTR("  -> fixedValue: %i"), (int)fixedValue);
    if(logQueue) communication_log_P(LEVEL_INFO, PSTR("  -> tile_x: %i"), (int)tile_x);
    if(logQueue) communication_log_P(LEVEL_INFO, PSTR("  -> tile_y: %i"), (int)tile_y);

    addTask_task(getTask_moveForward_oneTile(dir,
            speed, 
            tile_x,
            tile_y, 
            fixedValue
    ));
}