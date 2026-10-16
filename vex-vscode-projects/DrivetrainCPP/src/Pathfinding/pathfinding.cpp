#include "vex.h"
using namespace vex;

// To change what a unit of the coordinates represents, change these values
// bounds of the field
double MAX_X = 144;
double MIN_X = 0;
double MAX_Y = 144;
double MIN_Y = 0;
// what's the ratio between an x or y unit and an inch. eg. 1 if 1 y = 1 inch
double unitsToInchesRatio = 1;

std::vector<ObstacleBox> obstacleIDs = {};
std::vector<Point> path = {};

/*
follows path
@param speed how fast to move along the path
*/
void moveOnPath(){
    // perhaps it would be best to look into pure pursuit as well. might be good.
    // this only works with omni-wheels.
    while (!path.empty()){
        turnTo(path.front().x, path.front().y);
        move(pointDist(Odom::x, Odom::x, path.front().x, path.front().y) * unitsToInchesRatio, 40);
        Odom::setPosition(path.front().x, path.front().y);
        path.erase(path.begin());
    }
}

/*
turns the robot towards an absolute position
@param x x coordinate of point to turn to
@param y y coordinate of point to turn to
*/
void turnTo(double x, double y){
    // // convert point to degrees from 0 degrees
    // double correctRotation = rad2Deg(atan2(y - Odom::y, x - Odom::x));
    // if (correctRotation<0){
    //     correctRotation+=360;
    // }
    // // find the difference between current rotation and desired rotation
    // TODO: write angle subtract function
    // double turnAmount = angleSubtract(correctRotation,Odom::rotation);
    
    // // convert that amount of degrees into turns and turn that much.
    // turn(deg2Turn(turnAmount));
    // Odom::addRotation(turnAmount);
}

/*
creates a path to the position x,y and sets path to that new path.
If the path can't be made, does nothing.
uses A* search algorithm and adds on the theta* part but it's in a really lazy way.
Can probably be heavily optimized, maybe look for help on that?

@param startX the x coordinate to start from
@param startY the y coordinate to start from
@param endX the goal's x coordinate
@param endX the goal's y coordinate
*/
void findPath(double startX, double startY, double endX, double endY){
    bool canMove = true;
    double spotX = startX;
    double spotY = startY;
    double nextX;
    double nextY;
    double curNextVal;

    while ((spotX != endX || spotY != endY) && canMove){
        Point newPoint;
        newPoint.x = spotX;
        newPoint.y = spotY;
        path.push_back(newPoint);

        curNextVal = 2147483647;
        canMove = false;
        // direction = 0 means direction = NW, 1=N, etc.
        for (int direction = 0; direction < 9; direction++){
            if (direction == 4){
                continue;
            }
            // loops through immediate neighbors
            nextX = spotX + (floor(direction / 3.0) - 1);
            nextY = spotY + (direction % 3 - 1);
            // distance from current location to where you're testing if you want to go.
            double g = pointDist(spotX, spotY, nextX, nextY);
            // euclidian heuristic.
            double h = pointDist(nextX, nextY, endX, endY);
            // added together, f
            double f = g+h;
            // if you can go to a point and it's not already been gone to and the f is smaller than the existing one
            if (notBlocked(nextX, nextY) && notInCoordinateList(nextX, nextY, path) && f < curNextVal && isValid(nextX, nextY)){
                canMove = true;
                curNextVal = f;
            }
        }
        // update position
        spotX = nextX;
        spotY = nextY;
    }
    if (canMove){
        // this is where theta* comes in. 
        // instead of just having the grid based stuff, check for line of sight and remove stuff between.
        applyTheta();
        Brain.Screen.print("Pathing success!");
    }
    else{
        Brain.Screen.print("Pathing failed!");
    }
}

/*
removes points that aren't needed due to there being line of sight between the points it connects.
*/
void applyTheta(){
    std::vector<Point> tempList;
    for (int i = 1; i < path.size()-1; i++){
        if (!raycast(path.at(i-1), path.at(i+1))){
            tempList.push_back(path.at(i));
        }
    }
    tempList.push_back(path.at(path.size()));
    path.swap(tempList);
}

/*
checks for line of sight between two points
@param point1 starting point
@param point2 end point
*/
bool raycast(Point point1, Point point2){
    double curY = point1.y;
    double curX = point1.x;
    double magnitude = pointDist(point1.x,point1.y,point2.x,point2.y);
    while (!inRange(curY, point2.y, 1) || !inRange(curX, point2.x, 1)){
        if (!notBlocked(curX, curY)){
        return false;
        }
        curY += (point2.y-point1.y)/magnitude;
        curX += (point2.x-point1.x)/magnitude;
    }
    return true;
}

/*
calculates how far two points are from each other
@param x x coordinate of origin point
@param y y coordinate of origin point
@param x2 x coordinate of end point
@param y2 y coordinate of end point
*/
double pointDist(double x1, double y1, double x2, double y2){
    return sqrt(pow(x1-x2, 2) + pow(y1-y2, 2));
}


double pointDist(Point point1, Point point2){
    return sqrt(pow(point1.x-point2.x, 2) + pow(point1.y-point2.y, 2));
}
/*
checks if there's an obstacle at point x, y
@param x x position
@param y y position
*/
bool notBlocked(double x, double y){
    for (int cubeIndex = 0; cubeIndex < obstacleIDs.size(); cubeIndex++){
        if (obstacleIDs.at(cubeIndex).inBox(x, y)){
            return false;
        }
    }
    return true;
}

/*
checks if there's an obstacle at point x, y
@param myPoint double array with two elements {x, y}
*/
bool notBlocked(Point myPoint){
    for (int cubeIndex = 0; cubeIndex < obstacleIDs.size(); cubeIndex++){
        if (obstacleIDs.at(cubeIndex).inBox(myPoint)){
            return false;
        }
    }
    return true;
}

/*
checks if coordinates are in a list of coordinates
@param x x coordinate to check
@param y y coordinate to check
@param myList List to check for x,y in
*/
bool notInCoordinateList(double x, double y, std::vector<Point> myList){
    for (int index = 0; index < myList.size(); index++){
        if (myList.at(index).x == x && myList.at(index).y == y){
            return false;
        }
    }
    return true;
}

/* 
checks if a point is within the bounds of the field
@param x x coordinate
@param y y coordinate
*/
bool isValid(double x, double y){
    if (x>=MIN_X && x<=MAX_X && y>=MIN_Y && y<=MAX_Y){
        return true;
    }
    return false;
}

/* 
checks if a point is within the bounds of the field
@param myPoint point to check if it's valid
*/
bool isValid(Point myPoint){
    if (myPoint.x>=MIN_X && myPoint.x<=MAX_X && myPoint.y>=MIN_Y && myPoint.y<=MAX_Y){
        return true;
    }
    return false;
}
