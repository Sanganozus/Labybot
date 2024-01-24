#include <avr/io.h>

#include "initSensors.h"

void initDecoder() {
    //aktiviert PORTS
    PCICR |= (1 << PCIE1); //8-15
    PCICR |= (1 << PCIE0); //0-7

    //aktivierte PIN (erster Encoder)
    PCMSK1 |= (1 << PCINT12);
    PCMSK1 |= (1 << PCINT13);

    //aktivierte PIN (zweiter Encoder)
    PCMSK0 |= (1 << PCINT0);
    PCMSK0 |= (1 << PCINT1);
}

void initBumper() {
    //PIN DDD0 und DDD1 als Eingang
    DDRD &= ~(1<<DDD0) & ~(1<<DDD1);

    //PULL-UP Wiederstände
    PORTD |= (1<<PD0) | (1<<PD1);


    //prakitsch konfigurieren
    //INT0 pin bits zu 0-1, sendet Interrupt request wenn Flanke fällt
    EICRA |= (0<<ISC00);
    EICRA |= (1<<ISC01);

    //mit INT1 pin bits das gleiche
    EICRA |= (0<<ISC10);
    EICRA |= (1<<ISC11);


    //aktiviert softwareseitig die Interrupts!
    EIMSK |= (1<<INT0);
    //EIMSK |= (1<<INT1);
}