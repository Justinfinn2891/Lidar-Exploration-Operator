#include "../include/coordinates.h"
#include <cmath>

//Find the x based on two angles and a distance
float Coordinates::findZ(const float &verticleAngle, const float &horizontalAngle, const float &distance){
    return (distance * sin(verticleAngle) * sin(horizontalAngle));
}

//Find the y based on two angles and a distance
float Coordinates::findY(const float &verticleAngle, const float &horizontalAngle, const float &distance){
    return (distance * sin(verticleAngle) * cos(horizontalAngle)); 
}

//Find the z based on one angle and a distance
float Coordinates::findX(const float &verticleAngle, const float &distance){
    return (distance * cos(verticleAngle));
}