#include "vex.h"

using namespace vex;

double kp = 1.2;
double ki = 0.8;
double kd = 0.1;
double integralMax = 30;
double turnKp = 1;
double turnKi = 0;
double turnKd = 0;
static double speed = 40;
// i use this leniency variable for like... everything that need leniency. 
// I'd make more but I don't want to tweak more stuff...
static double leniency = 0.5;
static double correctionSensitivity = 0.5;
static double lookAhead = 1;
static double integratedError = 0;

/*
sets pid constants
@param kiVal integral constant
@param kpVal proportional constant
@param kdVal derivative constant
*/
void setConstants(double kpVal, double kiVal, double kdVal){
  kp = kpVal;
  ki = kiVal;
  kd = kdVal;
}


/* Calculates velocity.
Velocity = proportion + integral + derivative

p. constant * error (percent distance)

i. constant * integral of error (found over time via right riemann sum of error)

d. constant * derivative of error (slope from last error to current error)

Honestly, just look up what a PID is.

@param dist percent of the total distance needed to be traveled that has been traveled, 1% = .01
@param cycleTime total time that has passed since the last loop of the PID loop
@param errorDiff error in the last loop of the PID loop
*/
double PIDCalc(double distPercent, double cycleTime, double lastError){
  // 1 - dist is error.
  double proportion = (1 - distPercent) * kp;

  double integral = (integratedError) * ki;

  double derivative = (((1-distPercent) - lastError)/cycleTime) * kd;

  return integral + derivative + proportion;
}

double TurnPIDCalc(double distPercent, double cycleTime, double lastError){
  // 1 - dist is error.
  double proportion = (1 - distPercent) * turnKp;

  double integral = (integratedError) * turnKi;

  double derivative = (((1-distPercent) - lastError)/cycleTime) * turnKp;

  return integral + derivative + proportion;
}



/* 
moves robot forward or backward
takes inches parameter and moves that many inches
negative inches moves the robot backward, positive inches moves the robot forward
@param inch inches to move
*/
void move(double inch, double spd){
  speed = spd;
  resetPositions();
  Brain.Timer.clear();
  double loopTime = 0.05;
  // total distance moved
  double curPosTotal = 0.0;
  // extra velocity for right side
  double rExtra = 0;
  // extra velocity for left side
  double lExtra = 0;
  // negative if moving back, positive if moving forward
  double dirVal = inch/fabs(inch);
  // what the time on the Brain's timer was before
  double oldTime = 0;
  // what the last percentage distance from the goal was.
  double lastError = 1;
  // what value to set the velocity of the wheels to
  double velToSet = 0;

  while (!inRange(curPosTotal * turnDist, inch, leniency)){
    wait(loopTime, seconds);
    curPosTotal = (rPos(turns) + lPos(turns))/2;
    Brain.Screen.newLine();
    Brain.Screen.print(curPosTotal*turnDist);
    
    integratedError = std::min(30.0, integratedError+(loopTime)*(1-(curPosTotal * turnDist)/inch));
    // PID Velocity.
    velToSet = speed * PIDCalc((curPosTotal * turnDist)/inch, Brain.Timer.value()-oldTime, lastError) * dirVal;

    rExtra = ((velToSet-rExtra)-rVel()) * correctionSensitivity;
    lExtra = ((velToSet-lExtra)-lVel()) * correctionSensitivity;
  
    setAllVelocity(velToSet * dirVal + lExtra, velToSet * dirVal + rExtra);
    
    lastError = 1 - ((curPosTotal * turnDist)/inch);
  }
  setAllVelocity(0);
}

// moves robot 24 inches forward
void move(){
  move(24, 40);
}



/* turns the robot left or right
takes turns paramter and moves that amount of a full turn. 
positive turns moves clockwise/right, negative moves counterclockwise/left.
@param turnsVal how much the robot should turn. -0.5 < x < 0.5, but it will be simplified if you choose to make it outside of that domain.
*/
void turn(double turnsVal){
  resetPositions();
  Brain.Timer.clear();
  turnsVal = toSignedAngle(turnsVal, Turns);
  Brain.Screen.newLine();
  Brain.Screen.print("turning is happening.");
  Brain.Screen.newLine();
  double curPosTotal = 0.0;
  double end = turnsVal * fullRotationDist;
  Brain.Screen.print(end);
  Brain.Screen.print(fullRotationDist);
  Brain.Screen.print(turnsVal);

  double rExtra = 0;
  double lExtra = 0;
  double oldTime = 0;
  double lastError = 1;
  double ogRPos = rPos(turns);
  double ogLPos = lPos(turns);
  speed = 20;
  // >0 means turn right
  if (turnsVal > 0){
    while (!inRange(curPosTotal * turnDist, end, leniency)){
      wait(0.1, seconds);
      Brain.Screen.newLine();
      // subtract right position because left side needs to be turning backward to turn right
      curPosTotal = ((-(rPos(turns)-ogRPos) + (lPos(turns)-ogLPos))/2);

      double velToSet = speed * TurnPIDCalc((curPosTotal * turnDist)/end, Brain.Timer.value()-oldTime, lastError);
      Brain.Screen.newLine();
      Brain.Screen.print(velToSet);
      // rExtra = ((-velToSet-rExtra)-rVel()) * correctionSensitivity;
      // lExtra = ((velToSet-lExtra)-lVel()) * correctionSensitivity;
      // Brain.Screen.newLine();
      // Brain.Screen.print(velToSet);
      Brain.Screen.newLine();
      Brain.Screen.print(curPosTotal);
      if ((curPosTotal * turnDist)/end <= 0){
        Brain.Screen.print("FUCK");
        return;
      }

      if (velToSet <= 0){
        Brain.Screen.print("FUCK");
        return;
      }
      setLeftVelocity(velToSet);
      setRightVelocity(-velToSet);
      // setAllVelocity(velToSet+lExtra,-velToSet+rExtra);

      oldTime = Brain.Timer.value();
      lastError = 1 - ((curPosTotal*turnDist)/end);
    }
  }
  else{
    // while (!inRange(curPosTotal * turnDist, end, leniency)){
    //   wait(0.1, seconds);
    //   Brain.Screen.newLine();
    //   Brain.Screen.print("turn loop is happening (negative).");
    //   // subtract left position because left side needs to be turning backward to turn left
    //   curPosTotal = ((rPos(turns) - lPos(turns))/2);

    //   double velToSet = speed * PIDCalc((curPosTotal * turnDist)/end, Brain.Timer.value()-oldTime, lastError);

    //   rExtra = ((velToSet-rExtra)-rVel()) * correctionSensitivity;
    //   lExtra = ((-velToSet-lExtra)-lVel()) * correctionSensitivity;
    //   setAllVelocity(-velToSet+lExtra,velToSet+rExtra);

    //   oldTime = Brain.Timer.value();
    //   lastError = 1 - ((curPosTotal*turnDist)/end);
    // }
  }

  setAllVelocity(0);
}


// finds position after distance on a path.
// @param dist distance to travel along the path in inches
Point nextPoint(double dist){
  if (path.size() > 0){
    int i = 0;
    while (i < path.size()-1){
      // if distance argument is further than current two path points, move to the next two.
      if (dist > pointDist(path.at(i), path.at(i+1))){
        dist -= pointDist(path.at(i), path.at(i+1));
      }
      else{
        Point newPoint;
        // x difference / magnitude * distance
        newPoint.x = ((path.at(i).x-path.at(i+1).x)/pointDist(path.at(i), path.at(i+1))) * dist;
        // y difference / magnitude * distance
        newPoint.y = ((path.at(i).y-path.at(i+1).y)/pointDist(path.at(i), path.at(i+1))) * dist;
        return newPoint;
      }
      i++;
    }
    // if distance > the path's full distance, just return the final point in the path.
    Point newPoint;
    newPoint.x = path.at(path.size()-1).x;
    newPoint.y = path.at(path.size()-1).y;
    return newPoint;
  }
  // if there's nothing in path, return a point that's just where the robot is.
  Point newPoint;
  newPoint.x = Odom::x;
  newPoint.y = Odom::y;
  return newPoint;

}
// move smoothly, looking ahead at where the robot should be later along the path.
void purePursuit(){
  // this is specifically for pathfinding, so if there's no path it can't do anything.
  if (path.empty()){
    return;
  }
  Brain.Timer.clear();

  double curPosTotal = 0;
  double velToSet = speed;
  double totalDist;
  double oldTime = 0;
  // integrated as in this is the result of integration. We're going to integrate the acceleration.
  double integratedXVelocity = 0;
  double integratedYVelocity = 0;
  double totalDistance = 0;
  double startingRPosition = rPos(turns);
  double startingLPosition = lPos(turns);

  // final position, it's wherever the end of the path is
  double targetX = path.at(path.size()-1).x;
  double targetY = path.at(path.size()-1).y;

  // problem with current code: should probably rely on an odometer, but right now it's just hoping that the robot is never moved ever.
  while (path.size()){
    wait(0.05, seconds);
    curPosTotal = ((rPos(turns)-startingRPosition) + (lPos(turns)-startingLPosition))/2;
    // Point next = nextPoint(curPosTotal);

    // angle between target point and robot heading.
    double signedAngleDistance = toSignedAngle((rad2Deg(toPositiveAngle(atan2(targetY-Odom::y, targetX-Odom::x), Radians))-inertialSensor.heading()), Degrees);
    // negative angles are clockwise (right), 
    // so that means we need to make right velocity negative when difference in angles is negative
    // also I'm multiplying the distance value by 2 so that the robot will kind of turn in place at values >90
    double rightVelocity = speed*(1 + signedAngleDistance/90);
    // vice versa for left velocity.
    double leftVelocity = speed*(1 - signedAngleDistance/90);

    // lExtra = std::max(toSignedAngle(correctRotation-Odom::rotation, Degrees)/360 * speed * correctionSensitivity, 0.0);
    // rExtra = std::max(toSignedAngle(Odom::rotation-correctRotation, Degrees)/360 * speed * correctionSensitivity, 0.0);
    
    setAllVelocity(leftVelocity, rightVelocity);

    // riemann sums in programming by beloved. must convert from Gs to in/s^2, which is ~Gs*9.81*39
    integratedXVelocity += (inertialSensor.acceleration(xaxis)*386.1) * Brain.Timer.value()-oldTime;
    integratedYVelocity += (inertialSensor.acceleration(yaxis)*386.1) * Brain.Timer.value()-oldTime;

    if (pointDist(Odom::x, Odom::y, targetX, targetY) < lookAhead + leniency){
      totalDistance += pointDist(0, 0, integratedXVelocity, integratedYVelocity) * turnDist;
    }

    // add change in position to the stored x and y values. because fuck that $200 gps.
    Odom::addPosition(integratedXVelocity, integratedYVelocity);
    Point next = nextPoint(totalDistance + lookAhead);
    targetX = next.x;
    targetY = next.y;
    oldTime = Brain.Timer.value();
    Brain.Screen.newLine();
    Brain.Screen.print("X: ");
    Brain.Screen.print(Odom::x);
    Brain.Screen.newLine();
    Brain.Screen.print("Y: ");
    Brain.Screen.print(Odom::y);
    Brain.Screen.newLine();
    Brain.Screen.print("Target X: ");
    Brain.Screen.print(targetX);
    Brain.Screen.newLine();
    Brain.Screen.print("Target Y: ");
    Brain.Screen.print(targetY);
  }
}