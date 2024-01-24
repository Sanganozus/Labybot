#ifndef ISRCUSTOM_H
#define ISRCUSTOM_H

#include <math.h>
#include <stdint.h>
#include <avr/interrupt.h>


//Encoder
extern int16_t counter1Encoder;
extern uint16_t counter1Umdrehungen;
extern int16_t counter2Encoder;
extern uint16_t counter2Umdrehungen;

extern float encoder1MM;
extern float encoder2MM;
extern int16_t counter1EncoderBalancing;
extern int16_t counter2EncoderBalancing;


//Bumper 
extern uint16_t counter1Bumper;
//extern uint16_t counter2Bumper;
extern uint16_t bumper1Old;
//extern uint16_t bumper2Old;
extern uint8_t bumped;

extern float umfang;

extern uint8_t direction1;
extern uint8_t direction2;

/*
extern uint8_t pin0OldJ;
extern uint8_t pin1OldJ;
extern uint8_t pin0OldB;
extern uint8_t pin1OldB;
*/


uint16_t convertInfraredToMM(uint16_t x);

#endif