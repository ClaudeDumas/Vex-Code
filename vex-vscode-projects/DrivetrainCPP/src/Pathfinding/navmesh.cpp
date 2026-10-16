#include "vex.h"
using namespace vex;

/*
creates a new set of obstacle points
*/
void createField(){
    applyRadius(robotRadius);
    blockWalls(robotRadius);
}

/*
adds a box of obstacles from bottom left point 1 x,y to top right point 2 x,y
@param point1 bottom left point
@param point2 top right point
*/
void addObstacleBox(double point1[2], double point2[2]){
    ObstacleBox newBox;
    newBox.xCenter = (point1[1] + point2[1])/2.0;
    newBox.yCenter = (point1[2] + point2[2])/2.0;
    newBox.halfLength = (point2[2]-point1[2])/2.0;
    newBox.halfWidth = (point2[1]-point1[1])/2.0;
    obstacleIDs.push_back(newBox);
}

/*
adds a box of obstacles from bottom left point 1 x,y to top right point 2 x,y
@param x bottom left point x coordinate
@param y bottom left point y coordinate
@param x2 top right point x coordinate
@param y2 top right point y coordinate
*/
void addObstacleBox(double x, double y, double x2, double y2){
    ObstacleBox newBox;
    newBox.xCenter = (x + x2)/2.0;
    newBox.yCenter = (y + y2)/2.0;
    newBox.halfLength = (y2-y)/2.0;
    newBox.halfWidth = (x2-x)/2.0;
    obstacleIDs.push_back(newBox);
}

/*
creates obstacles next to the edges of bounds
@param radius how far from the edges the obstacles will be
*/
void blockWalls(double radius){
    /*
    for visual effect:
    XOO
    XOO
    XOO
    */
    addObstacleBox(MIN_X,MIN_Y,radius,MAX_Y);
    /*
    XOO
    XOO
    XXX
    */
    addObstacleBox(radius,MIN_Y,MAX_X,radius);
    /*
    XXX
    XOO
    XXX
    */
    addObstacleBox(radius,MAX_Y-radius,MAX_X, MAX_Y);
    /*
    XXX
    XOX
    XXX
    */
    addObstacleBox(MAX_X-radius,radius,MAX_X, MAX_Y-radius);
}

/*
adds obstacle boxes protruding from already existing obstacles based on a radius
eg. box at 2,2, radius 1, add boxes at 1,2, 3,2, 2,1, 2,3, 3,3, 1,1, 1,3, and 3,1
@param radius how far out from original obstacles the added obstacles should be.
*/
void applyRadius(double radius){
    std::vector<ObstacleBox> originalObstacleIDs;
    for (int i = 0; i < obstacleIDs.size(); i++){
        originalObstacleIDs.push_back(obstacleIDs.at(i));
    }
    
    for (int index = 0; index < originalObstacleIDs.size(); index++){
        addObstacleBox(originalObstacleIDs[index].xCenter - radius - originalObstacleIDs[index].halfWidth, originalObstacleIDs[index].yCenter - radius - originalObstacleIDs[index].halfLength, originalObstacleIDs[index].xCenter + radius + originalObstacleIDs[index].halfWidth, originalObstacleIDs[index].yCenter + radius + originalObstacleIDs[index].halfLength);
    }
}

bool ObstacleBox::inBox(Point myPoint){
    if (inRange(myPoint.x, xCenter, halfWidth) && inRange(myPoint.y, yCenter, halfLength)){
        return true;
    }
    return false;
}

bool ObstacleBox::inBox(double x, double y){
    if (inRange(x, xCenter, halfWidth) && inRange(y, yCenter, halfLength)){
        return true;
    }
    return false;
}