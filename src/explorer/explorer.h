#ifndef EXPLORER_H
#define EXPLORER_H

/**
 * Beginnt die Explorierung des Labyrinths
*/
void startExploring();

bool isExploring();

void stopExploring();


/**
 * Beinhaltet einen TIMETASK, welcher sich um die Explorierung des Labyrinths kümmert
*/
void explore(void);

#endif