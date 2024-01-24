#include "path.h"

#include <tools/timeTask/timeTask.h>
#include <communication/communication.h>
#include <pathFollower/pathFollower.h>
#include "../driving/driving.h"
#include "../pose/pose.h"
#include "../channels/channels.h"
#include "../tasks/tasks.h"
#include "main.h"
#include "math.h"
#include "motor/motor.h"

#include <avr/pgmspace.h>           // AVR Program Space Utilities



/**
 * Gibt ausgehend von der aktuellen Pose einen Fahrbefehl zurück, welcher den Roboter auf den Lookahead-Punkt zufahren lässt
*/
void calculateDriveCommand(Pose_t *pose, const FPoint_t *lookahead) {
    
    int l = achsenlaenge;
    float v = 1800.0f;


    float Angle = atan2((lookahead->y)-pose->y, (lookahead->x)-pose->x);

    Angle -= pose->theta;

    if (Angle < -M_PI)
        Angle += 2*M_PI;

    if (Angle > M_PI)
        Angle -= 2*M_PI;

    
    float vDiff = 18.0f*Angle*l/2.0f;
    float vr = v + vDiff;
    float vl = v - vDiff;


    Motor_setPWM(-vl, -vr);
}





void checkPath() {
    TIMETASK(FOLLOWER_TASK, 10) {
            const PathFollowerStatus_t* pathFollower_status = pathFollower_getStatus();
            if (pathFollower_status->enabled) {
                if (pathFollower_update(getPose())) {
                    calculateDriveCommand(getPose(), &pathFollower_status->lookahead);
                } else {
                    stopDrive();
                }
                sendPathFollowerStatus(pathFollower_status); // send pathFollower_status on channel CH_OUT_PATH_FOLLOW_STATUS
            }
    }
}


