#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>


#include "vex.h"

using namespace vex;

// Brain should be defined by default
brain Brain;


// START V5 MACROS
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
// END V5 MACROS


// Robot configuration code.
digital_out Clamp = digital_out(Brain.ThreeWirePort.H);
controller Controller1 = controller(primary);



// generating and setting random seed
void initializeRandomSeed(){
  int systemTime = Brain.Timer.systemHighResolution();
  double batteryCurrent = Brain.Battery.current();
  double batteryVoltage = Brain.Battery.voltage(voltageUnits::mV);

  // Combine these values into a single integer
  int seed = int(batteryVoltage + batteryCurrent * 100) + systemTime;

  // Set the seed
  srand(seed);
}



void vexcodeInit() {

  //Initializing random seed.
  initializeRandomSeed(); 
}


// Helper to make playing sounds from the V5 in VEXcode easier and
// keeps the code cleaner by making it clear what is happening.
void playVexcodeSound(const char *soundName) {
  printf("VEXPlaySound:%s\n", soundName);
  wait(5, msec);
}



// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

#pragma endregion VEXcode Generated Robot Configuration

// ----------------------------------------------------------------------------
//                                                                            
//    Project:                                               
//    Author:
//    Created:
//    Configuration:        
//                                                                            
// ----------------------------------------------------------------------------

motor FrontRightDrive = motor(PORT2, ratio6_1, false);

motor FrontLeftDrive = motor(PORT19, ratio6_1, false);

motor BackRightDrive = motor(PORT10, ratio6_1, false);

motor BackLeftDrive = motor(PORT20, ratio6_1, false);

motor Intake = motor(PORT3, ratio6_1, false);

motor Conveyor = motor(PORT1, ratio6_1, false);
// Include the V5 Library
#include "vex.h"

// Allows for easier use of the VEX Library
using namespace vex;

// Begin project code
void resetPositions(){
  FrontRightDrive.setPosition(0,turns);
  BackRightDrive.setPosition(0,turns);
  FrontLeftDrive.setPosition(0,turns);
  BackLeftDrive.setPosition(0,turns);
}

void move(double inch){
  resetPositions();

  FrontRightDrive.spin(forward);
  BackRightDrive.spin(forward);
  FrontLeftDrive.spin(forward);
  BackLeftDrive.spin(forward);

  bool movingForward = inch > 0;
  double curPosTotal = 0.0;
  double wheelDiam = 3.25;
  double pi = 3.1415;
  double gearRatio = 3/5;
  double turnDist = wheelDiam * pi * gearRatio;
  double speed = 100;
  double minModifier = 0.1;
  double modifier = 1.0;
  double tooFarDistInches = 0.1;

  // should try to work with these to adjust one side to the other.
  double rightMod = 1.0;
  double leftMod = 1.0;

  if (movingForward){
    while (curPosTotal * turnDist < inch || curPosTotal * turnDist > inch+tooFarDistInches){
        
      curPosTotal = (-(FrontRightDrive.position(turns) + BackRightDrive.position(turns)) + (FrontLeftDrive.position(turns) + BackLeftDrive.position(turns)))/4;

      // 100% speed minus the percent distance the robot has gone, decreases speed as it continues.
      if ((curPosTotal * turnDist) / inch > minModifier or (curPosTotal * turnDist) / inch < -minModifier){
        modifier = (1 - ((curPosTotal * turnDist) / inch));
      }
      else if ((curPosTotal * turnDist) / inch < 0){
        modifier = -minModifier;
      }
      else{
        modifier = minModifier;
      }

      // put together all the modifiers and change velocity.
      FrontRightDrive.setVelocity((speed * rightMod * modifier + 1) * -1, percent);
      BackRightDrive.setVelocity((speed * rightMod * modifier + 1) * -1, percent);
      FrontLeftDrive.setVelocity((speed * leftMod * modifier + 1), percent);
      BackLeftDrive.setVelocity((speed * leftMod * modifier + 1), percent);

      wait(20, msec);
    }
  }
  else{
    while (curPosTotal * turnDist > inch || curPosTotal * turnDist < inch-tooFarDistInches){
      curPosTotal = (-(FrontRightDrive.position(turns) + BackRightDrive.position(turns)) + (FrontLeftDrive.position(turns) + BackLeftDrive.position(turns)))/4;

      // 100% speed minus the percent distance the robot has gone, decreases speed as it continues.
      if ((curPosTotal * turnDist) / inch > minModifier or (curPosTotal * turnDist) / inch < -minModifier){
        modifier = (1 - ((curPosTotal * turnDist) / inch));
      }
      else if ((curPosTotal * turnDist) / inch < 0){
        modifier = -minModifier;
      }
      else{
        modifier = minModifier;
      }

      // put together all the modifiers and change velocity.
      FrontRightDrive.setVelocity((speed * rightMod * modifier), percent);
      BackRightDrive.setVelocity((speed * rightMod * modifier), percent);
      FrontLeftDrive.setVelocity((speed * leftMod * modifier) * -1, percent);
      BackLeftDrive.setVelocity((speed * leftMod * modifier) * -1, percent);
    }
  }
  FrontRightDrive.setVelocity(0, percent);
  BackRightDrive.setVelocity(0, percent);
  FrontLeftDrive.setVelocity(0, percent);
  BackLeftDrive.setVelocity(0, percent);
}


void turnLeft(double Turns){
  double fullTurnTurns = 360/55;
  double speed = 50;
  BackRightDrive.spinFor(forward, Turns * fullTurnTurns, turns);
  BackLeftDrive.spinFor(forward, Turns * fullTurnTurns, turns);
  FrontRightDrive.spinFor(forward, Turns * fullTurnTurns, turns);
  FrontLeftDrive.spinFor(forward, Turns * fullTurnTurns, turns);
}

void turnRight(double Turns){
  double fullTurnTurns = 360/55;
  double speed = 50;
  BackRightDrive.spinFor(reverse, Turns * fullTurnTurns, turns);
  BackLeftDrive.spinFor(reverse, Turns * fullTurnTurns, turns);
  FrontRightDrive.spinFor(reverse, Turns * fullTurnTurns, turns);
  FrontLeftDrive.spinFor(reverse, Turns * fullTurnTurns, turns);
}

void moveForward2(double inch){
  double wheelDiam = 3.25;
  double pi = 3.1415;
  double gearRatio = 3/5;
  double turnDist = wheelDiam * pi * gearRatio;
  double speed = 40.0;

  BackRightDrive.spinFor(forward, inch/turnDist, turns);
  BackLeftDrive.spinFor(reverse, inch/turnDist, turns);
  FrontRightDrive.spinFor(forward, inch/turnDist, turns);
  FrontLeftDrive.spinFor(reverse, inch/turnDist, turns);
}

void moveBackward2(double inch){
  double wheelDiam = 3.25;
  double pi = 3.1415;
  double gearRatio = 3/5;
  double turnDist = wheelDiam * pi * gearRatio;
  double speed = 40.0;

  BackRightDrive.spinFor(reverse, inch/turnDist, turns);
  BackLeftDrive.spinFor(forward, inch/turnDist, turns);
  FrontRightDrive.spinFor(reverse, inch/turnDist, turns);
  FrontLeftDrive.spinFor(forward, inch/turnDist, turns);
}


void preAutonomous(void) {
  // actions to do when the program starts
  Brain.Screen.clearScreen();
  Brain.Screen.print("pre auton code");
  wait(1, seconds);
}

void autonomous(void) {
  Brain.Screen.clearScreen();
  Brain.Screen.print("autonomous code");
  move(10);
  move(-10);
  // place automonous code here
}

void userControl(void) {
  Brain.Screen.clearScreen();
  double wheelRatio = 6;
  
  FrontRightDrive.setStopping(coast);
  BackRightDrive.setStopping(coast);
  FrontLeftDrive.setStopping(coast);
  BackLeftDrive.setStopping(coast);

  BackRightDrive.spin(forward);
  BackLeftDrive.spin(forward);
  FrontRightDrive.spin(forward);
  FrontLeftDrive.spin(forward);
  
  Intake.spin(forward);
  
  Conveyor.spin(forward);
  // place driver control in this while loop
  while (true) {
    wait(20, msec);

    double posR = (BackRightDrive.position(turns) + FrontRightDrive.position(turns))/2;
    double posL = (BackLeftDrive.position(turns) + FrontLeftDrive.position(turns))/2;
    
    double rExtra = 0;
    double lExtra = 0;

    if (posR>posL){
      rExtra += -1;
      lExtra += 1;
    }
    else if (posL>posR){
      lExtra += -1;
      rExtra += 1;
    }

    double rVelocity = (Controller1.Axis1.position()*(-1) - Controller1.Axis3.position()) * (0.7) + rExtra;
    double lVelocity = (Controller1.Axis1.position() - Controller1.Axis3.position()) * -0.7 + lExtra;
    if (Controller1.ButtonA.pressing()){
      Clamp.set(true);
    }
    if (Controller1.ButtonB.pressing()){
      Clamp.set(false);
    }

    Intake.setVelocity((Controller1.ButtonL1.pressing()-Controller1.ButtonR1.pressing()) * 100, percent);

    Conveyor.setVelocity((Controller1.ButtonL2.pressing()-Controller1.ButtonR2.pressing()) * 100, percent);
    
    BackLeftDrive.setVelocity(lVelocity, percent);

    FrontLeftDrive.setVelocity(lVelocity, percent);

    BackRightDrive.setVelocity(rVelocity, percent);

    FrontRightDrive.setVelocity(rVelocity, percent);

    resetPositions();
  }
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  // create competition instance
  competition Competition;

  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(userControl);

  // Run the pre-autonomous function.
  preAutonomous();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}