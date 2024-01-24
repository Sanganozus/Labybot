#include "sensors.h"
#include "main.h"

#include "initSensors.h"
#include "ISRCustom.h"
#include <communication/communication.h>
#include <tools/timeTask/timeTask.h>

#include <avr/io.h>       // AVR IO ports
#include <stdint.h>  
#include <io/adc/adc.h>
#include <util/atomic.h>



//----- Für das Balancing -----//
void resetCounts() {
    counter1EncoderBalancing = 0;
    counter2EncoderBalancing = 0;
}


int getCount_leftWheel() {
    return counter2EncoderBalancing * korrekturLinkesRad;
}

int getCount_rightWheel() {
    return counter1EncoderBalancing * korrekturRechtesRad;
}





uint8_t getBumperCount() {
    return bumped;
}

void incrementBumped() {
    bumped++;
}




void getAndResetEncoders(int16_t* encoder1, int16_t* encoder2) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        *encoder1 = counter1Encoder;
        *encoder2 = counter2Encoder;
        counter1Encoder = 0;
        counter2Encoder = 0;
    }
}

int16_t getEncoderVal1() {
    return counter1Encoder;
}

int16_t getEncoderVal2() {
    return counter2Encoder;
}


float getEncoder1MM() {
    return encoder1MM;
}

float getEncoder2MM() {
    return encoder2MM;
}

void setEncoder1MM(float val) {
    encoder1MM = val;
}

void setEncoder2MM(float val) {
    encoder2MM = val;
}

//nur zum Debuggen benutzen, direction hat keinen Einfluss auf tatsächliche Direction, da diese ja von den Encodern bestimmt wird.
uint8_t getDirection1() {
    return direction1;
}

uint8_t getDirection2() {
    return direction2;
}

uint16_t getCounterBumper() {
    return counter1Bumper;
}

uint16_t getCounterBumperOld() {
    return bumper1Old;
}

void setOldBumper(uint16_t val) {
    bumper1Old = val;
}




void initSensors(){
    initDecoder();
    initBumper();
}

void checkBumped(){
    TIMETASK(BUMPER_TASK, 800) {    
            if (counter1Bumper - bumper1Old >= 10) {
                bumped++;
                bumper1Old = counter1Bumper;
            }
        }
}

void updateTelemetry(){
    TIMETASK(TELEMETRY_TASK, 300) { // execute block approximately every 300ms
            // send telemetry data to HWPCS
            Telemetry_t telemetry;
            telemetry.bumpers.value = 0; // initialize with zero
            telemetry.bumpers.bitset.bit1 = 1;
            telemetry.contacts = bumped;
            telemetry.encoder1 = getEncoderVal1();
            telemetry.encoder2 = getEncoderVal2();
            telemetry.infrared1 = convertInfraredToMM(ADC_getFilteredValue(1)); //links
            telemetry.infrared2 = convertInfraredToMM(ADC_getFilteredValue(0)); //rechts
            telemetry.infrared3 = convertInfraredToMM(ADC_getFilteredValue(2)); //vorne
            telemetry.user1 = 20;
            telemetry.user2 = 42.42f;
            communication_writePacket(CH_OUT_TELEMETRY, (uint8_t*)&telemetry, sizeof(telemetry));
        }
}