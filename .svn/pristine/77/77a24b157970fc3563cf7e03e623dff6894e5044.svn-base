#include "main.h"

#include <io/uart/uart.h>
#include <communication/communication.h>
#include <tools/timeTask/timeTask.h>
#include <tools/powerSaver.h>
#include <io/led/led.h>
#include <motor/motor.h>
#include <pathFollower/pathFollower.h>
#include <io/adc/adc.h>

#include <avr/io.h>                 // AVR IO ports
#include <avr/interrupt.h>          // AVR Interrupts
#include <avr/pgmspace.h>           // AVR Program Space Utilities
#include <math.h>                   // Math functions and constants
#include <inttypes.h>               // Integer type conversions for printf, scanf and alike
#include <stdlib.h>

#include "tasks/taskManagement.h"

#include "driving/driving.h"
#include "sensors/sensors.h"
#include "channels/channels.h"
#include "pose/pose.h"
#include "path/path.h"
#include "explorer/explorer.h"
#include "tests/test.h"

//Zum Debuggen
#include "explorer/robot.h"


//**************************//
//----- Hilfsvariablen -----//
//**************************//

//Debug-Hilfsvariablen
bool logPose = 0;
bool logTelemetry = 0;
bool logQueue = 0;
bool logExplorer = 0;
bool logBalancing = 0;
bool logDist = 0;
bool logPoseCorrection = 0;
bool logCoordinatesCorrection = 0;

bool switch_poseCorrection = 0;
bool switch_coordinatesCorrection = 1;


float distGreen_rotateLeft = 0.0f;
float distGreen_rotateRight = 0.0f;
float distGreen_rotateBackward = 0.0f;

float distGreenRed_rotateLeft = 0.0f;
float distGreenRed_rotateRight = 0.0f;
float distGreenRed_rotateBackward = 0.0f;

bool debugContinue = 0;

//Für das Balancing
float korrekturLinkesRad = 0.98;
float korrekturRechtesRad = 1;

float achsenlaenge = 132.0f;
float correctionValue = 1.15f;
float tolerance_fixedValue = 5.0f;


//****************************//
//****************************//
//----- Internal Methods -----//
//****************************//


float getCorrectionValue_noRotate(){
    float correctionValue_noRotate = 20.0f;

    if(logCoordinatesCorrection) communication_log_P(LEVEL_INFO, PSTR(""));
    if(logCoordinatesCorrection) communication_log_P(LEVEL_INFO, PSTR("----- getCorrectionValue_noRotate. correctionValue_noRotate: %.3f -----"), correctionValue_noRotate);
    return 10.0f;
}

float getCorrectionValue_rotateLeft(){
    //float correctionValue_rotateLeft = sqrt(pow(distGreen_rotateLeft, 2) - pow(correctionValue_noRotate, 2))
    //+ distGreenRed_rotateLeft;;

    float correctionValue_rotateLeft = 25.0f;

    if(logCoordinatesCorrection) communication_log_P(LEVEL_INFO, PSTR(""));
    if(logCoordinatesCorrection) communication_log_P(LEVEL_INFO, PSTR("----- getCorrectionValue_rotateLeft. correctionValue_rotateLeft: %.3f -----"), correctionValue_rotateLeft);
    return 0;
}

float getCorrectionValue_rotateRight(){
    //float correctionValue_rotateRight = sqrt(pow(distGreen_rotateRight, 2) - pow(correctionValue_noRotate, 2))
    //+ distGreenRed_rotateRight;

    float correctionValue_rotateRight = 25.0f;

    if(logCoordinatesCorrection) communication_log_P(LEVEL_INFO, PSTR(""));
    if(logCoordinatesCorrection) communication_log_P(LEVEL_INFO, PSTR("----- getCorrectionValue_rotateRight. correctionValue_rotateRight: %.3f -----"), correctionValue_rotateRight);
    return 0;
}

float getCorrectionValue_rotateBackward(){
    //float correctionValue_rotateBackward = distGreen_rotateBackward - correctionValue_noRotate
    //+ distGreenRed_rotateBackward;

    float correctionValue_rotateBackward = 40.0f;

    if(logCoordinatesCorrection) communication_log_P(LEVEL_INFO, PSTR(""));
    if(logCoordinatesCorrection) communication_log_P(LEVEL_INFO, PSTR("----- getCorrectionValue_rotateBackward. correctionValue_rotateBackward: %.3f -----"), correctionValue_rotateBackward);
    return 0;
}

float getCorrectionValue_rotate(uint8_t dir){
    if(logCoordinatesCorrection) communication_log_P(LEVEL_INFO, PSTR(""));
    if(logCoordinatesCorrection) communication_log_P(LEVEL_INFO, PSTR("----- getCorrectionValue_rotate. dir:%s -----"), cardStr((Direction_t)dir));
    switch((Direction_t)dir){
        case FORWARD:
            return getCorrectionValue_noRotate();
        case LEFT:
            return getCorrectionValue_rotateLeft();
        case BACKWARD:
            return getCorrectionValue_rotateBackward();
        case RIGHT:
            return getCorrectionValue_rotateRight();
        default:
            return 0.0f;
    }
}


// initialization
static void init(void) {
    powerSaver_init(); // must be the first call!
    LED_init();
    uart_init();
    ADC_init(false);
    communication_init();
    pathFollower_init();
 
    // register communication callback functions which are executed by
    // communication_readPackets() in main loop when a packet is received from
    // HWPCS on the corresponding communication channel

	communication_setCallback(CH_IN_DEBUG, commDebug);
    communication_setCallback(CH_IN_DRIVE, commDrive);
	communication_setCallback(CH_IN_USER_COMMAND, commUserCommand);
	communication_setCallback(7, commTweak); //Zum Tasks hinzufügen
    communication_setCallback(CH_IN_POSE, poseUpdateAprilTag);
    communication_setCallback(CH_IN_ROBOT_PARAMS, commParameters);


    Motor_init();
    timeTask_init();

    //aktiviere die Pins sowie die Ports für die Encoder und Bumper
    initSensors();

    // global interrupt enable
    sei();

}

char *cardStr2(Direction_t dir){
	switch(dir){
		case DIRECTION_NORTH:
			return "North";
		case DIRECTION_EAST:
			return "East";
		case DIRECTION_SOUTH:
			return "South";
		case DIRECTION_WEST:
			return "West";
	}
	return "UNDEFINED";
}

uint16_t bumpedBefore = 0;

int main(void) {
    init();

    communication_log_P(LEVEL_INFO, PSTR("Booted"));

    communication_log(LEVEL_INFO, "Tests:");
    testAll();

    logQueue = 1;

    //request April Tag Pose to update first location
    GetPose_t * requestPoseAprilTag = (GetPose_t*) malloc(sizeof(GetPose_t));
    requestAprilTagPose(requestPoseAprilTag);
    free(requestPoseAprilTag);

    // do forever
    for (;;) {
        timeTask_RequestAprilTag();

        //Time Tasks für die Sensoren
        checkBumped();
        updateTelemetry();

        //Alles was mit der Taskqueue zu tun hat
        manageTasks();

        //Zum Ausgleichen der Räder
        checkBalancing();
        checkPath();

        //Zum Erkunden des Labyrinths
        explore();


        communication_readPackets();

        //log the bumper and encoder data
        TIMETASK(DEBUG_TELEMETRY, 1000) {
            if(getBumperCount() != bumpedBefore){
                communication_log_P(LEVEL_INFO, PSTR("bumped: %i"), getBumperCount());
                bumpedBefore = getBumperCount();
            }

            if(logTelemetry){
                communication_log_P(LEVEL_INFO, PSTR("EncoderVal1: %i"), getEncoderVal1());
                communication_log_P(LEVEL_INFO, PSTR("EncoderVal2: %i"), getEncoderVal2());

                communication_log_P(LEVEL_INFO, PSTR("count_leftWheel: %i"), getCount_leftWheel());
                communication_log_P(LEVEL_INFO, PSTR("count_rightWheel: %i"), getCount_rightWheel());

                communication_log_P(LEVEL_INFO, PSTR("encoder1MM: %i"), getEncoder1MM());
                communication_log_P(LEVEL_INFO, PSTR("encoder2MM: %i"), getEncoder2MM());

                

                communication_log_P(LEVEL_INFO, PSTR("-------------------"));
            }
        }


        TIMETASK(LED_TASK, 500) { // execute block approximately every 500ms
            LED2_TOGGLE();
        }

        

        TIMETASK(POSE_TASK, 100) { // execute block approximately every 150ms
            //update the Pose (basierend auf Encoder-Werten)
            poseUpdate();

            if(logPose) communication_log_P(LEVEL_INFO, PSTR("first April Tag Update received: %i"), firstAprilTagUpdate());

            //send pose update to HWPCS
            communication_writePacket(CH_OUT_POSE, (uint8_t*)getPose(), sizeof(*getPose()));
        }

    }

    return 0;
}