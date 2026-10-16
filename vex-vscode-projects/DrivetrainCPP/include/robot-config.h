using namespace vex;

extern brain Brain;

extern inertial inertialSensor;

extern digital_out Output;

extern digital_out Hood;

extern controller Controller1;

extern motor FrontRightDrive;

extern motor FrontLeftDrive;

extern motor BackRightDrive;

extern motor BackLeftDrive;

extern motor IndexerBack;
extern motor Storage;
extern motor IndexerFrontBottom;
extern motor IndexerFrontTop;

extern motor rightWheelMotors[2];

extern motor leftWheelMotors[2];

// gear ratio of wheels
extern double gearRatio;
// diameter of wheels
extern double wheelDiam;
// how far the robot moves per full turn of a wheel
extern double turnDist;
// half of whatever your robot's largest diameter is. Eg. your robot is 12x18, robotRadius is 9.
extern double robotRadius;

// how far the robot's wheels need to turn to spin the robot around 360 degrees
extern double fullRotationDist;
