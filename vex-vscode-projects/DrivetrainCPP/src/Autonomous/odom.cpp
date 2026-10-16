#include "vex.h"

using namespace vex;

double Odom::x;
double Odom::y;
/* 
sets the robot's absolute position
@param x x coordinate
@param y y coordinate
*/
void Odom::setPosition(double x, double y){
    Odom::x = x;
    Odom::y = y;
}

/* 
sets the robot's rotation
@param deg heading in degrees
*/
void Odom::setRotation(double deg){
    Odom::rotation = deg;
}  

void Odom::addPosition(double x, double y){
    Odom::x += x;
    Odom::y += y;
}

// add to rotation
void Odom::addRotation(double deg){
    Odom::rotation += deg;
    while (Odom::rotation > 360){
        Odom::rotation -= 360;
    }
    while (Odom::rotation < 0){
        Odom::rotation += 360;
    }
}

