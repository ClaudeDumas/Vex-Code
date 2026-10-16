#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>


#include "vex.h"

using namespace vex;


// START V5 MACROS
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
// END V5 MACROS


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


// Begin project code


void preAutonomous(void) {
  // actions to do when the program starts
  Brain.Screen.clearScreen();
  Brain.Screen.print("pre auton code");
  wait(1, seconds);
  Odom::setPosition(0, 0);
}

void autonomous(void) {
  spinWheels();
  FrontLeftDrive.setStopping(coast);
  FrontRightDrive.setStopping(coast);
  BackLeftDrive.setStopping(coast);
  BackRightDrive.setStopping(coast);
  IndexerBack.spin(forward);
  IndexerFrontBottom.spin(forward);
  IndexerFrontTop.spin(forward);
  IndexerBack.setVelocity(0, percent);
  IndexerFrontBottom.setVelocity(0, percent);
  IndexerFrontTop.setVelocity(0, percent);
  Brain.Screen.clearScreen();
  Brain.Screen.print("autonomous code");
  turn(0.5);
  // move(27, 40);
  // setAllVelocity(0);
  // move(19, 10);
  // spinToTopGoal();
  // place automonous code here
}

void userControl(void) {
  // Brain.Screen.clearScreen();

  driveLoop();
  // MUST CALIBRATE: kp, ki, kd, full rotation distance
  // calibrationLoop();
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