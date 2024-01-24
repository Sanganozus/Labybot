#include <inttypes.h>
#include <math.h>

#include "ISRCustom.h"



//init the values
int16_t counter1Encoder = 0;
int16_t counter2Encoder = 0;


int16_t counter1EncoderBalancing = 0;
int16_t counter2EncoderBalancing = 0;


float encoder1MM = 0;
float encoder2MM = 0;
uint16_t counter1Umdrehungen = 0;
uint16_t counter2Umdrehungen = 0;


uint16_t counter1Bumper = 0;

uint16_t bumper1Old = 0;
uint8_t bumped = 0;

float umfang = 3 * M_PI;


//Richtung des Roboters für die Pose: 0 ist vorwärts, 1 ist zurück
uint8_t direction1 = 0;
uint8_t direction2 = 0;


//Variablen für die Pins
static uint8_t pin0OldJ = 0;
static uint8_t pin1OldJ = 0;
static uint8_t pin0OldB = 0;
static uint8_t pin1OldB = 0;



//converts Infrared-Signal to distance in mm
uint16_t convertInfraredToMM(uint16_t x) {
    uint16_t mapping = 2.137*100 - 3.76e-1*x + 1.984e-4*x*x;
    return mapping;
}





//aus den Flanken die Richtung der Encoder ableiten
static uint8_t getDirection(uint8_t pin0, uint8_t pin1, uint8_t pin0Old, uint8_t pin1Old) {
    uint8_t dir;

    if ((pin0 > 0 && pin1 > 0) && (pin0Old > 0 && pin1Old == 0)) {
        //vorwärts
        dir = 0;
    } else if ((pin0 == 0 && pin1 > 0) && (pin0Old > 0 && pin1Old > 0)) {
        dir = 0;
    } else if ((pin0 == 0 && pin1 == 0) && (pin0Old == 0 && pin1Old > 0)) {
        dir = 0;
    } else if ((pin0 > 0 && pin1 == 0) && (pin0Old == 0 && pin1Old == 0)) {
        dir = 0;
    } else {
        //rückwärts
        dir = 1;
    }
    return dir;
}





//Interrupt-Service Routinen
ISR(PCINT0_vect) {

    //auslesen
    uint8_t currentValPB0 = (PINB & (1 << PB0)); //=> PINB ist 8BIT=1Byte groß enthält alle PINS im Port B, PB0 == 0
    uint8_t currentValPB1 = (PINB & (1 << PB1));

    uint8_t dir = getDirection(currentValPB0, currentValPB1, pin0OldB, pin1OldB);


    if (dir == 0) {
        //hier muss die Direction vertauscht werden, da wahrscheinlich die Aktuatoren falsch eingebaut sind!
        direction1 = 1;
        counter1Encoder--;
        counter1EncoderBalancing--;
    } else {
        direction1 = 0;
        counter1Encoder++;
        counter1EncoderBalancing++;
    }

    pin0OldB = currentValPB0;
    pin1OldB = currentValPB1;
}



ISR(PCINT1_vect) {

    //auslesen
    uint8_t currentValPJ3 = (PINJ & (1 << PJ3)); //=> PJ3 == 3
    uint8_t currentValPJ4 = (PINJ & (1 << PJ4));

    uint8_t dir = getDirection(currentValPJ3, currentValPJ4, pin0OldJ, pin1OldJ);  

    if (dir == 0) {
        direction2 = 0;
        counter2Encoder++;
        counter2EncoderBalancing++;
    } else {
        direction2 = 1;
        counter2Encoder--;
        counter2EncoderBalancing--;
    }

    pin0OldJ = currentValPJ3;
    pin1OldJ = currentValPJ4;
}


ISR(INT0_vect) {
    counter1Bumper++;
}

