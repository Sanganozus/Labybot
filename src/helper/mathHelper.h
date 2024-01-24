#include <stdbool.h>

/**
 * @param angleA erster Winkel (zwischen -Pi und +Pi)
 * @param angleB zweiter Winkel (zwischen -Pi und +Pi)
 * 
 * @returns true: Falls der erste Winkel gegen den Uhrzeigersinn vom zweiten Winkel liegt
*/
bool checkAngle_greater(float angleA, float angleB);

/**
 * @param angleA erster Winkel (zwischen -Pi und +Pi)
 * @param angleB zweiter Winkel (zwischen -Pi und +Pi)
 * 
 * @returns true: Falls der erste Winkel im Uhrzeigersinn vom zweiten Winkel liegt
*/
bool checkAngle_lower(float angleA, float angleB);

float angles_add(float startAngle, float addAngle);

float angle_subtract(float angleA, float angleB);

float getDistance(float x_0, float y_0, float x_1, float y_1);