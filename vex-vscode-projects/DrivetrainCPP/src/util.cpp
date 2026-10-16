#include "vex.h"

using namespace vex;



// sets every wheel motor's position to 0
void resetPositions(){
    FrontRightDrive.setPosition(0,turns);
    BackRightDrive.setPosition(0,turns);
    FrontLeftDrive.setPosition(0,turns);
    BackLeftDrive.setPosition(0,turns);
}

// returns average velocity of right side wheel motors
double lVel(){
    double result = 0;
    int i = 0;
    for (motor myMotor : leftWheelMotors){
        result += myMotor.velocity(percent);
        i++;
    }
    return result/(double)(i);
}

// returns average velocity of right side wheel motors
double rVel(){
    double result = 0;
    int i = 0;
    for (motor myMotor : rightWheelMotors){
        result += myMotor.velocity(percent);
        i++;
    }
    return result/(double)(i);
}

// returns average position of right side wheel motors
// @param units Units of the position
double rPos(vex::rotationUnits units){
    double result = 0;
    int i = 0;
    for (motor myMotor : rightWheelMotors){
        result += myMotor.position(units);
        i++;
    }
    return result/(double)(i);
}

// returns average position of left side wheel motors
// @param units Units of the position
double lPos(vex::rotationUnits units){
    double result = 0;
    int i = 0;
    for (motor myMotor : leftWheelMotors){
        result += myMotor.position(units);
        i++;
    }
    return result/(double)(i);
}
// returns average position of wheel motors
// @param units Units of the position
double wheelsPos(vex::rotationUnits units){
    return (rPos(units) + lPos(units))/2;
}
  
/* change velocity of every wheel motor
@param lVelocity velocity of left wheels
@param rVelocity velocity of right wheels
*/
void setAllVelocity(double lVelocity, double rVelocity){
    setRightVelocity(rVelocity);
    setLeftVelocity(lVelocity);
}
  
/* change velocity of every wheel motor
@param velocity the velocity to set all wheels to
*/
void setAllVelocity(double velocity){
    setRightVelocity(velocity);
    setLeftVelocity(velocity);
}

/* set velocity of all right side wheels
@param velocity the velocity to change all right wheels to
*/
void setRightVelocity(double velocity){
    FrontRightDrive.setVelocity(velocity, percent);
    BackRightDrive.setVelocity(velocity, percent);
}

/* set velocity of all left side wheels
@param velocity the velocity to change all left wheels to
*/
void setLeftVelocity(double velocity){
    FrontLeftDrive.setVelocity(velocity, percent);
    BackLeftDrive.setVelocity(velocity, percent);
}

/* returns if a value is with leniency value of the base value
@param value the value to test
@param base the value you're checking if the first value is close to
@param leniency how far value can be from base and still return true
*/
bool inRange(double value, double base, double leniency){
    if (value > base-leniency && value < base+leniency){
        return true;
    }
    return false;
}

/* moves wheels to a set position
@param rEndPos position to move right weels to
@param lEndPos position to move left wheels to
@param leniency how far from the destination it can be
@param speed how fast it moves
*/
void moveTo(double rEndPos, double lEndPos, double leniency, double speed){
    while (!inRange(rPos(turns), rEndPos, leniency) || !inRange(lPos(turns), lEndPos, leniency)){
        // if rPos isn't in the right place, correct it
        if (!inRange(rPos(turns), rEndPos, leniency)){
            setRightVelocity(speed * ((rPos(turns)-rEndPos)/fabs((rPos(turns)-rEndPos))));
        }

        // if lPos isn't in the right place, correct it
        if (!inRange(lPos(turns), lEndPos, leniency)){
            setRightVelocity(speed * ((lPos(turns)-lEndPos)/fabs((lPos(turns)-lEndPos))));
        }
    }
    setAllVelocity(0);
}

/* moves wheels to a set position
@param rEndPos position to move right weels to
@param lEndPos position to move left wheels to
@param leniency how far from the destination it can be
*/
void moveTo(double rEndPos, double lEndPos, double leniency){
    while (!inRange(rPos(turns), rEndPos, leniency) || !inRange(lPos(turns), lEndPos, leniency)){
        // if rPos isn't in the right place, correct it
        if (!inRange(rPos(turns), rEndPos, leniency)){
            setRightVelocity(10 * ((rPos(turns)-rEndPos)/fabs((rPos(turns)-rEndPos))));
        }

        // if lPos isn't in the right place, correct it
        if (!inRange(lPos(turns), lEndPos, leniency)){
            setRightVelocity(10 * ((lPos(turns)-lEndPos)/fabs((lPos(turns)-lEndPos))));
        }
    }
    setAllVelocity(0);
}

// converts radians to degrees
double rad2Deg(double rad){
    return rad * (180.0/pi);
}

// converts degrees to radians
double deg2Rad(double deg){
    return deg * (pi/180.0);
}

// converts degrees to turns
double deg2Turn(double deg){
    return deg/360;
}

// returns angle between two 2D vectors.
double vectorAngle(double vector1[2], double vector2[2]){
    double V1Mag = pointDist(vector1[0], vector1[1], 0, 0);
    double V2Mag = pointDist(vector2[0], vector2[1], 0, 0);
    return acos((vector1[0] * vector2[0] + vector1[1] * vector2[1])/(V1Mag*V2Mag));
}

/* converts an angle in degrees into a simplified signed angle in degrees
post-condition: converted angle returned, -180 <= angle <= 180
@param angle angle that is converted, unit is degrees.
*/
double toSignedAngle(double angle, RotationUnit unit){
    switch (unit){
        case Turns:
            while (angle > 0.5){
                angle -= 1;
            }
            while (angle < -0.5){
                angle += 1;
            }
            break;
        case Degrees:
            while (angle > 180){
                angle -= 360;
            }
            while (angle < -180){
                angle += 360;
            }
            break;
        case Radians:
            while (angle > pi){
                angle -= 2*pi;
            }
            while (angle < -pi){
                angle += 2*pi;
            }
            break;
    }
    return angle;
}

double toPositiveAngle(double angle, RotationUnit unit){
    switch (unit){
        case Turns:
            while (angle > 1){
                angle -= 1;
            }
            while (angle < 0){
                angle += 1;
            }
            break;
        case Degrees:
            while (angle > 360){
                angle -= 360;
            }
            while (angle < 0){
                angle += 360;
            }
            break;
        case Radians:
            while (angle > 2*pi){
                angle -= 2*pi;
            }
            while (angle < 0){
                angle += 2*pi;
            }
            break;
    }
    return angle;
}

void spinToTopGoal(){
    IndexerBack.setVelocity(100, percent);
    IndexerFrontBottom.setVelocity(-100, percent);
    IndexerFrontTop.setVelocity(-100, percent);
}
void spinToMiddleGoal(){
    IndexerBack.setVelocity(100, percent);
    IndexerFrontBottom.setVelocity(-100, percent);
    IndexerFrontTop.setVelocity(100, percent);
}
void spinToStorage(){
    // Storage.setVelocity(100, percent);
    IndexerBack.setVelocity(-100, percent);
    IndexerFrontTop.setVelocity(-100, percent);
}
void spinOutStorage(){
    // Storage.setVelocity(-100, percent);
}
void spinOut(){
    IndexerBack.setVelocity(-100, percent);
    IndexerFrontBottom.setVelocity(100, percent);
    IndexerFrontTop.setVelocity(100, percent);
}
void StorageToTopGoal(){
    IndexerBack.setVelocity(100, percent);
    IndexerFrontBottom.setVelocity(-100, percent);
    IndexerFrontTop.setVelocity(-100, percent);
    // Storage.setVelocity(-100, percent);
}

void spinWheels(){
    FrontRightDrive.spin(forward);
    BackRightDrive.spin(forward);
    FrontLeftDrive.spin(forward);
    BackLeftDrive.spin(forward);
}




// constant pi
double pi = 3.14159265358979;