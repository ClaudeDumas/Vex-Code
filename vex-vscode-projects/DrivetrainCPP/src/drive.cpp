#include "vex.h"

using namespace vex;

// the value to set whatever you're calibrating
static double calibrationVal = 0.1;
// percent power on the drivetrain
static double driveModifier = 0.8;
// how sensitive the robot is to correcting the velocity of the wheels.
static double correctionSensitivity = 0;
static double turnModifier = 0.7;

/*
driver control, need to put in all your  settings here.
Button activated things go under "button toggles"
Motor controlled things go under "motor control"
*/
void driveLoop(){
  // added to velocity to adjust wheels to the correct real velocity
  double rExtra = 0;
  double lExtra = 0;

  // this doesn't really do anything because of the adjustment to the wheels forcing the velocity to 0, 
  // but in some scenario it might help with the motors not burning out.
  FrontRightDrive.setStopping(coast);
  BackRightDrive.setStopping(coast);
  FrontLeftDrive.setStopping(coast);
  BackLeftDrive.setStopping(coast);

  // of course need to make the wheels move, place other motor stuff here as well.
  BackRightDrive.spin(forward);
  BackLeftDrive.spin(forward);
  FrontRightDrive.spin(forward);
  FrontLeftDrive.spin(forward);

  IndexerBack.spin(forward);
  Storage.spin(forward);
  IndexerFrontBottom.spin(forward);
  IndexerFrontTop.spin(forward);
  IndexerBack.setStopping(coast);
  Storage.setStopping(coast);
  IndexerFrontTop.setStopping(coast);
  IndexerFrontBottom.setStopping(coast);


  while (true) {
    wait(20, msec);
    IndexerFrontTop.setVelocity(0, percent);
    IndexerFrontBottom.setVelocity(0, percent);
    Storage.setVelocity(0, percent);
    IndexerBack.setVelocity(0, percent);

    double rVelocity = (Controller1.Axis3.position() - (Controller1.Axis1.position())*turnModifier) * driveModifier + rExtra;
    double lVelocity = (Controller1.Axis3.position() + (Controller1.Axis1.position())*turnModifier) * driveModifier + lExtra;

    // desired velocity minus real velocity of side. 
    // rExtra = (((rVelocity-rExtra)-rVel())) * correctionSensitivity;
    // lExtra = ((lVelocity-lExtra)-lVel()) * correctionSensitivity;

    // button toggles
    if (Controller1.ButtonR1.pressing()){
      spinToTopGoal();
    }
    else if (Controller1.ButtonR2.pressing()){
      spinToMiddleGoal();
    }
    else if(Controller1.ButtonB.pressing()){
      spinOut();
    }
    else if (Controller1.ButtonL1.pressing()){
      spinToStorage();
    }
    else if (Controller1.ButtonL2.pressing()){
      spinOutStorage();
    }
    else if (Controller1.ButtonDown.pressing()){
      Output.set(false);
      Brain.Screen.newLine();
      Brain.Screen.print("Turned False.");
    }
    else if (Controller1.ButtonUp.pressing()){
      Output.set(true);
      Brain.Screen.newLine();
      Brain.Screen.print("Turned True.");
    }
    else if (Controller1.ButtonRight.pressing()){
      Hood.set(true);
    }
    else if (Controller1.ButtonLeft.pressing()){
      Hood.set(false);
    }
    else if (Controller1.ButtonA.pressing()){
      StorageToTopGoal();
    }

    // motor controls

    BackLeftDrive.setVelocity(lVelocity, percent);

    FrontLeftDrive.setVelocity(lVelocity, percent);

    BackRightDrive.setVelocity(rVelocity, percent);

    FrontRightDrive.setVelocity(rVelocity, percent);
  }
}

// increase calibration val by 0.1
void increaseCaliVal(){
  calibrationVal+=0.1;
  // currently setting drive constants.
  setConstants(1.3, calibrationVal, 0);
  Brain.Screen.print(calibrationVal);
  Brain.Screen.newLine();
}

// decrease calibration val by 0.1
void decreaseCaliVal(){
  calibrationVal-=0.1;
  setConstants(1.3, calibrationVal, 0);
  Brain.Screen.print(calibrationVal);
  Brain.Screen.newLine();
}

/*
a driver control to callibrate a value. have to go to drive.cpp to change what it is.
currently: extra velocity to apply to each side of the drivetrain.
*/
void calibrationLoop(){
  FrontRightDrive.setStopping(coast);
  BackRightDrive.setStopping(coast);
  FrontLeftDrive.setStopping(coast);
  BackLeftDrive.setStopping(coast);

  BackRightDrive.spin(forward);
  BackLeftDrive.spin(forward);
  FrontRightDrive.spin(forward);
  FrontLeftDrive.spin(forward);

  setAllVelocity(0);

  Controller1.ButtonUp.pressed(increaseCaliVal);
  Controller1.ButtonDown.pressed(decreaseCaliVal);
  Controller1.ButtonLeft.pressed(move);

}

void motor1Button(vex::controller::button onButton, vex::motor myMotor){
  myMotor.setVelocity(onButton.pressing() * 100, percent);
}

void motor2Button(vex::controller::button onButton, vex::controller::button offButton, vex::motor myMotor){
  myMotor.setVelocity((onButton.pressing() - offButton.pressing()) * 100, percent);
}

void toggle(vex::controller::button onButton, vex::controller::button offButton, vex::digital_out dOut){
  if (onButton.pressing()){
    dOut.set(true);
  }
  if (offButton.pressing()){
    dOut.set(false);
  }
}
