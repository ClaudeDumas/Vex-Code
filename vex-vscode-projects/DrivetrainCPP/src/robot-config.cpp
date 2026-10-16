#include "vex.h"

using namespace vex;

brain Brain;

digital_out Output = digital_out(Brain.ThreeWirePort.A);
digital_out Hood = digital_out(Brain.ThreeWirePort.H);

inertial inertialSensor = inertial(PORT2);

controller Controller1 = controller(primary);

// wheels 
// when you change this you also need to change setRightVelocity and setLeftVelocity.

// right side of wheels
motor FrontRightDrive = motor(PORT20, ratio6_1, false);

motor BackRightDrive = motor(PORT19, ratio6_1, false);

// left side of wheels
motor FrontLeftDrive = motor(PORT12, ratio6_1, true);

motor BackLeftDrive = motor(PORT11, ratio6_1, true);

motor rightWheelMotors[2] = {FrontRightDrive, BackRightDrive};
motor leftWheelMotors[2] = {FrontLeftDrive, BackLeftDrive};

// motor Intake = motor(PORT3, ratio6_1, false);

// motor Conveyor = motor(PORT1, ratio6_1, false);

motor IndexerBack = motor(PORT9, ratio18_1, false);
motor Storage = motor(PORT10, ratio18_1, false);
motor IndexerFrontBottom = motor(PORT8, ratio18_1, false);
motor IndexerFrontTop = motor(PORT1, ratio18_1, false);

// gear ratio of wheels
double gearRatio = 0.6;
// diameter of wheels
double wheelDiam = 3.25;
// how far the robot moves per full turn of a wheel
double turnDist = pi * wheelDiam * gearRatio;
// half of whatever your robot's largest diameter is. Eg. your robot is 12x18, robotRadius is 9.
double robotRadius = 9;

// Length of the distance between both sides of the drivetrain in inches
double wheelRectangleWidth = 12.75;

/* 
this value is: 4a * INTEGRAL 0 to pi/2 (sqrt(a^2*cos^2(x)+b^2*sin^2(x))dx), the perimeter of an ellipse
a is the semi-major axis of the ellipse your wheels make when turning. This would generally be top left wheel to bottom right wheel or bottom left wheel to top right wheel.
b is the semi-minor axis of the ellipse your wheels make when turning.

here's an image:
https://i.ytimg.com/vi/ubT_wqjXdNI/maxresdefault.jpg

finding b is tricky.
TODO: find b
*/

// how far the robot's wheels need to turn to spin the robot around 360 degrees in inches
double fullRotationDist = 45;