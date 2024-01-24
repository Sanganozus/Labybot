#ifndef CHANNELS_H
#define CHANNELS_H

#include <stdint.h>
#include <communication/packetTypes.h>

//******************//
/*
Aufgabe: 
Kümmert sich um die Kommunikation mit dem Roboter auf den Kanälen.
*/
//******************//

void commDebug(const uint8_t* a, __attribute__((unused)) const uint16_t b);

void commDrive(const uint8_t* a, __attribute__((unused)) const uint16_t b);

void commUserCommand(const uint8_t* a, __attribute__((unused)) const uint16_t b);

void commParameters(const uint8_t* a, __attribute__((unused)) const uint16_t b);

void commTweak(const uint8_t* a, __attribute__((unused)) const uint16_t b);

void sendPathFollowerStatus(const PathFollowerStatus_t* pathFollower_status);

#endif /* CHANNELS_H */