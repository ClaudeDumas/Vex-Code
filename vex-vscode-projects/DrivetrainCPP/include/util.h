void resetPositions();
enum RotationUnit{
    Turns,
    Degrees,
    Radians
};
double rPos(vex::rotationUnits units);
double lPos(vex::rotationUnits units);
double rVel();
double lVel();
double wheelsPos(vex::rotationUnits units);
void setAllVelocity(double lVelocity, double rVelocity);
void setAllVelocity(double velocity);
void setRightVelocity(double velocity);
void setLeftVelocity(double velocity);
bool inRange(double value, double base, double leniency);
void moveTo(double rEndPos, double lEndPos, double leniency);
void moveTo(double rEndPos, double lEndPos, double leniency, double speed);
double rad2Deg(double rad);
double deg2Rad(double deg);
double deg2Turn(double deg);
double toSignedAngle(double angle, RotationUnit unit);
double toPositiveAngle(double angle, RotationUnit unit);
double vectorAngle(double vector1[2], double vector2[2]);
void spinOut();
void spinToTopGoal();
void spinToMiddleGoal();
void spinToStorage();
void spinOutStorage();
void StorageToTopGoal();
void spinWheels();

extern double pi;