#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <geometry_msgs/PointStamped.h>
#include <iostream>
#include <fstream>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>

#include <math.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

class PointStar
{
public:
    int x; // In Pixels
    int y; // In Pixels
    int parentx;
    int parenty;
    int gcost;
    int hcost;
    int fcost;

    PointStar(int newX, int newY)
    {
        x = newX;
        y = newY;
    }

    PointStar() {}
};

struct ArrayPaths
{
    std::vector<PointStar> path;
    double initPointX, initPointY, endPointX, endPointY; // In metters
};

class AStar
{
public:
    int startPosition_x; // In metters
    int startPosition_y; // In metters
    int endPosition_x; // In metters
    int endPosition_y; // In metters

    nav_msgs::OccupancyGrid::ConstPtr map;
    int** mapArray;
    int** mapArrayState;
    int mapTolerance = 8;
    float mapOrigin_x;
    float mapOrigin_y;
    float mapResolution;
    int mapWidth;
    int mapHeight;

    std::vector<PointStar> pathAStar;
    std::vector<PointStar> pathAStarSelectedPoints;
    std::vector<PointStar> pathAStarDense;
    std::vector<PointStar> pathAStarSmoothed;

    AStar() = default;

    AStar(nav_msgs::OccupancyGrid::ConstPtr newMap);

    void getMapArray();

    void getPath();

    void getPoints(std::vector<PointStar> inputVector);

    void getDensePath(std::vector<PointStar> inputVector);

    void getSmoothedPath(std::vector<PointStar> inputVector);

    unsigned long long int smoothFormula(int n, int i);
    int getCost(int startx, int starty, int endx, int endy);
    void applyAStar(int newStartPosition_x, int newStartPosition_y, int newEndPosition_x, int newEndPosition_y);

private:
};
