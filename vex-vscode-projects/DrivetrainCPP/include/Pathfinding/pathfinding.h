#include "navmesh.h"
class Point{
    public:
        double x;
        double y;
};

class ObstacleBox{
    public:
        double xCenter;
        double yCenter;
        double halfWidth;
        double halfLength;
        bool inBox(Point myPoint);
        bool inBox(double x, double y);
};

extern std::vector<ObstacleBox> obstacleIDs;
extern std::vector<Point> path;

/*
note that the field is 12 ft x 12 ft or 144 in x 144 in
*/
extern double MAX_X;
extern double MIN_X;
extern double MAX_Y;
extern double MIN_Y;

// moving functions
void moveOnPath();
void turnTo(double x, double y);

// pathing functions
void findPath(double startX, double startY, double endX, double endY);
void applyTheta();

// checking validity of points
bool isValid(double x, double y);
bool isValid(Point myPoint);
bool notInCoordinateList(double x, double y, std::vector<Point> myList);
bool notBlocked(Point myPoint);
bool notBlocked(double x, double y);

// vector stuff
bool raycast(Point point1, Point point2);
double pointDist(double x1, double y1, double x2, double y2);
double pointDist(Point point1, Point point2);

