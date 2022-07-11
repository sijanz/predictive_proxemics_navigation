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
    int endPosition_x;   // In metters
    int endPosition_y;   // In metters

    //nav_msgs::OccupancyGrid::ConstPtr map;
    std::vector<std::vector<int>> map{};
    //int **mapArray;
    //int **mapArrayState;
    std::vector<std::vector<int>> mapArray_v{};
    std::vector<std::vector<int>> mapArrayState_v{};
    int mapTolerance = 8;
    float mapOrigin_x;
    float mapOrigin_y;
    float mapResolution;
    int mapWidth;
    int mapHeight;

    std::vector<PointStar> pathAStar{};
    std::vector<PointStar> pathAStarSelectedPoints{};
    std::vector<PointStar> pathAStarDense{};
    std::vector<PointStar> pathAStarSmoothed{};

    AStar() = default;

    AStar(const std::vector<std::vector<int>>& newMap);

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
