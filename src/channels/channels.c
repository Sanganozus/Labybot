#include "channels.h"
#include "../main.h"
#include "../driving/driving.h"
#include "../tasks/taskManagement.h"
#include "../sensors/sensors.h"
#include "../explorer/labyrinthState.h"
#include "../explorer/robot.h"
#include "../pose/pose.h"
#include "tools/labyrinth/labyrinth.h"
#include "../explorer/explorer.h"
#include "../sensors/vision.h"

#include <motor/motor.h>

#include <communication/communication.h>
#include <avr/pgmspace.h> //Für PSTR (PSTR: static String pointer)
#include <inttypes.h>
#include <math.h>


void commDebug(__attribute__((unused)) const uint8_t* packet, const uint16_t size) { //CH00 Debug
    communication_log(LEVEL_FINE, "received %" PRIu16 " bytes", size);
    communication_log_P(LEVEL_INFO, PSTR("Booted"));
}

void commDrive(const uint8_t* packet, __attribute__((unused)) const uint16_t size) { //CH01 Drive
    DriveCommand_t* cmd = (DriveCommand_t*) packet;
    communication_log_P(LEVEL_INFO, PSTR("Drive Packet arrived!"));


    initDrive(cmd->speed, cmd->steering);

    communication_log_P(LEVEL_INFO, PSTR("Speed: %i, Steering: %i"), cmd->speed, cmd->steering);
}

// callback function for communication channel CH_IN_USER_COMMAND (User Command View in HWPCS)
void commUserCommand(const uint8_t* packet, __attribute__((unused)) const uint16_t size) {
    UserCommand_t* cmd = (UserCommand_t*) packet;
    communication_log_P(LEVEL_INFO, PSTR("Commands: %i"), cmd -> id);

    switch (cmd->id) {
        case 0: // command ID 0: stop motors
            stopDrive();
            resetCounts();
            //Motor_stopAll();
            break;
        case 1: // command ID 1: drive forwards with balancing
            initDrive_withBalancing(pose_getCurrentCardinalDirection(), 3000, 0);
            break;
        case 2: // command ID 2: drive backwards with balancing
            initDrive_withBalancing(pose_getCurrentCardinalDirection(), -3000, 0);
            break;
        case 3: // command ID 3: Queue (On/Off)
            if(logQueue){
                communication_log_P(LEVEL_INFO, PSTR("Queue Switch, queue size: %i"), getTaskQueueSize());
            }

            if(!isTaskQueueIterating()) {
                if(logQueue){
                    communication_log_P(LEVEL_INFO, PSTR("Queue On"));
                }
                startQueue(pose_getCurrentCardinalDirection());
            } else {
                if(logQueue){
                    communication_log_P(LEVEL_INFO, PSTR("Queue Off"));
                }
                stopQueue();
            }

            //logFlags();
            communication_log_P(LEVEL_INFO, PSTR("isTaskQueueIterating: %i"), isTaskQueueIterating());
            break;
        case 4: // command ID 4: exploring On/Off
            if(!isExploring()){
                startExploring();
            } else {
                stopExploring();
            }
            communication_log_P(LEVEL_INFO, PSTR("exploring: %i"), isExploring());
            break;
        case 5: // command ID 5: Next Move - Explorer
            debugContinue = 1;
            communication_log_P(LEVEL_INFO, PSTR("debugContinue: %i"), debugContinue);
            break;
        case 6: // command ID 6: Switch Log Balancing
            logBalancing = !logBalancing;
            communication_log_P(LEVEL_INFO, PSTR("logBalancing: %i"), logBalancing);
            break;
        case 7: // command ID 7: Switch Log Queue
            logQueue = !logQueue;
            communication_log_P(LEVEL_INFO, PSTR("logQueue: %i"), logQueue);
            break;
        case 8: // command ID 8: Switch Log Pose
            logPose = !logPose;
            communication_log_P(LEVEL_INFO, PSTR("logPose: %i"), logPose);
            break;
        case 9: // command ID 9: Switch Log Telemetry
            logTelemetry = !logTelemetry;
            communication_log_P(LEVEL_INFO, PSTR("logTelemetry: %i"), logTelemetry);
            break;
        case 10: // command ID 10: Switch Log Explorer
            logExplorer = !logExplorer;
            communication_log_P(LEVEL_INFO, PSTR("logExplorer: %i"), logExplorer);
            break;
        case 11: // command ID 11: Manual Skip of current Task
            if(logQueue) {
                communication_log_P(LEVEL_INFO, PSTR("Manual skip"));
            }
            skipTask();
            break;
        case 12: // command ID 12: Reset Queue
            resetQueue();
            break;
        case 13: // command ID 13: drive forwards without balancing
            initDrive(3000, 0);
            break;
        case 14: // command ID 14: drive backwards without balancing
            initDrive(-3000, 0);
            break;
        case 15: // command ID 15: rotate clockwise
            initDrive(3000, 3000);
            break;
        case 16: // command ID 16: rotate counter clockwise
            initDrive(-3000, -3000);
            break;
        case 17: // command ID 17: Check Debug
            communication_log_P(LEVEL_INFO, PSTR("Labyrinth cell 2 north == cell 1 south: %i"), labyrinth_getWallIndex(0, 0, DIRECTION_SOUTH) == labyrinth_getWallIndex(1, 0, DIRECTION_NORTH));
            break;
        case 18: // command ID 18: rotate by angle test
            break;
        case 19: // command ID 19: thetaDiff reset
            startMeasuring_thetaDiff();
            break;
        case 20: // command ID 20: enqueue rotate_clockwise
            enqueue_rotateToCardinalDirection(1500, robot_orientationAfterRotation(RIGHT));
            break;
        case 21: // command ID 21: enqueue rotate_counterClockwise
            enqueue_rotateToCardinalDirection(1500, robot_orientationAfterRotation(LEFT));
            break;
        case 22: // command ID 22: enqueue rotate north
            enqueue_rotateToCardinalDirection(1500, DIRECTION_NORTH);
            break;
        case 23: // command ID 23: enqueue rotate east
            enqueue_rotateToCardinalDirection(1500, DIRECTION_EAST);
            break;
        case 24: // command ID 24: enqueue rotate south
            enqueue_rotateToCardinalDirection(1500, DIRECTION_SOUTH);
            break;
        case 25: // command ID 25: enqueue rotate west
            enqueue_rotateToCardinalDirection(1500, DIRECTION_WEST);
            break;
        case 26: // command ID 26: enqueue rotate forward
            enqueue_rotateToCardinalDirection(1500, robot_orientationAfterRotation(FORWARD));
            break;
        case 27: // command ID 27: enqueue rotate right
            enqueue_rotateToCardinalDirection(1500, robot_orientationAfterRotation(RIGHT));
            break;
        case 28: // command ID 28: enqueue rotate backward
            enqueue_rotateToCardinalDirection(1500, robot_orientationAfterRotation(BACKWARD));
            break;
        case 29: // command ID 29: enqueue rotate left
            enqueue_rotateToCardinalDirection(1500, robot_orientationAfterRotation(LEFT));
            break;
        case 30: // command ID 30: enqueue moveForward one tile
            enqueue_moveForward_oneTile(1500, pose_getCurrentCardinalDirection());
            break;
        case 31: 
            logDist = !logDist;
            communication_log_P(LEVEL_INFO, PSTR("logDist: %i"), logDist);
            break;
        case 32: // command ID 32: Test Balancing with fixed value
            initDrive_withBalancing_withFixedValue(pose_getCurrentCardinalDirection(), 2000, 0, 20.0f);
            break;
        case 33: // command ID 33: switch_poseCorrection
            switch_poseCorrection = !switch_poseCorrection;
            communication_log_P(LEVEL_INFO, PSTR("poseCorrection: %i"), switch_poseCorrection);
            break;
        case 34: // command ID 34: switch_coordinatesCorrection
            switch_coordinatesCorrection = !switch_coordinatesCorrection;
            communication_log_P(LEVEL_INFO, PSTR("coordinatesCorrection: %i"), switch_coordinatesCorrection);
            break;
        case 35:
            logPoseCorrection = !logPoseCorrection;
            communication_log_P(LEVEL_INFO, PSTR("logPoseCorrection: %i"), logPoseCorrection);
            break;
        case 36: // command ID 36: switch_coordinatesCorrection
            logCoordinatesCorrection = !logCoordinatesCorrection;
            communication_log_P(LEVEL_INFO, PSTR("logCoordinatesCorrection: %i"), logCoordinatesCorrection);
            break;
        case 40: // command ID 40: getCurrentCardinalDirection
            communication_log_P(LEVEL_INFO, PSTR("currentCardinalDirection: %s"), cardStr(pose_getCurrentCardinalDirection()));
            break;
        case 41: // command ID 41: isWalls
            communication_log_P(LEVEL_INFO, PSTR("isWall: forward: %i, left: %i right: %i"), isWall(FORWARD), isWall(LEFT), isWall(RIGHT));
            communication_log_P(LEVEL_INFO, PSTR("visited: forward: %i, left: %i, backward: %i, right: %i"), checkVisited_inDirection(robot_getRow(), robot_getColumn(), FORWARD), checkVisited_inDirection(robot_getRow(), robot_getColumn(), LEFT), checkVisited_inDirection(robot_getRow(), robot_getColumn(), BACKWARD), checkVisited_inDirection(robot_getRow(), robot_getColumn(), RIGHT));
            break;
        case 42: // command ID 42: getRow & getColumn
            communication_log_P(LEVEL_INFO, PSTR("column: %i, row: %i"), robot_getColumn(), robot_getRow());
            break;
    }
}

void commParameters(const uint8_t* packet, __attribute__((unused)) const uint16_t size){
    RobotParameters_t* cmd = (RobotParameters_t*) packet;
    communication_log_P(LEVEL_INFO, PSTR("Parameters"));

    achsenlaenge = cmd->axleWidth;
    correctionValue = cmd->user1;
    tolerance_fixedValue = cmd->user2;

    communication_log_P(LEVEL_INFO, PSTR("achsenlaenge: %.3f, correctionValue: %.3f, tolerance_fixedValue: %.3f"), achsenlaenge, correctionValue, tolerance_fixedValue);
}

// callback function for adding tasks to the queue
void commTweak(const uint8_t* packet, __attribute__((unused)) const uint16_t size) {
    TaskCommand_t* cmd = (TaskCommand_t*) packet;
    communication_log_P(LEVEL_INFO, PSTR("TaskCommand"));

    uint8_t korrLinks = cmd->korrLinks;
    uint8_t korrRechts = cmd->korrRechts;

    uint16_t tolerance_fixedValue = cmd->tolerance_fixedValue;
    uint16_t correctionValue = cmd->correctionValue;

    uint16_t tolerance_theta = cmd->tolerance_theta;
    uint8_t breakTime = cmd->breakTime;

    setToleranceValues(tolerance_fixedValue, tolerance_theta, correctionValue);
    setBreakTime((uint16_t)breakTime*100);

    korrekturLinkesRad = 1.0f - (float)korrLinks/100.0f;
    korrekturRechtesRad = 1.0f - (float)korrRechts/100.0f;

    communication_log_P(LEVEL_INFO, PSTR("korrekturLinkesRad: %i, korrekturRechtesRad: %i"), (int)(korrekturLinkesRad*100), (int)(korrekturRechtesRad*100));
}

void sendPathFollowerStatus(const PathFollowerStatus_t* pathFollower_status) {
    communication_writePacket(CH_OUT_PATH_FOLLOW_STATUS, (const uint8_t *)pathFollower_status, sizeof(*pathFollower_status));
}

