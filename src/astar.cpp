#include <math.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include "predictive_proxemics_navigation/astar.h"


AStar::AStar(nav_msgs::OccupancyGrid::ConstPtr newMap)
{
    // mapResolution = 20.0;

    /*
    startPosition_x = newStartPosition_x;
    startPosition_y = newStartPosition_y;
    endPosition_x = newEndPosition_x;
    endPosition_y = newEndPosition_y;
    */

    map = newMap;
    
    mapOrigin_x = map->info.origin.position.x;
    mapOrigin_y = map->info.origin.position.y;
    mapResolution = map->info.resolution;
    mapWidth = map->info.width;
    mapHeight = map->info.height;


    // mapOrigin_x = 0;
    // mapOrigin_y = 0;
    // mapWidth = map.at(0).size();
    // mapHeight = map.size();

    // DEBUG
    // std::cout << "start position x: " << startPosition_x << std::endl;
    // std::cout << "start position y: " << startPosition_y << std::endl;
    // std::cout << "end position x: " << endPosition_x << std::endl;
    // std::cout << "end position y: " << endPosition_y << std::endl;
    // std::cout << "map width: " << mapWidth << std::endl;
    // std::cout << "map heigth: " << mapHeight << std::endl;
    // std::cout << "getting map array...\n";

    getMapArray();

    // DEBUG
    // std::cout << "getMapArray() finished\n";

    // getPath();

    // DEBUG
    // std::cout << "getPath() finished\n";

    // getDensePath(pathAStar);

    // DEBUG
    // std::cout << "getDensePath() finished\n";

    // getPoints(pathAStarDense);

    // DEBUG
    // std::cout << "getPoints() finished\n";

    // getSmoothedPath(pathAStarSelectedPoints);

    // DEBUG
    /*
    std::cout << "map array:\n";
    for (int x = 0; x < mapWidth; ++x) {
        for (int y = 0; y < mapHeight; ++y) {
            std::cout << mapArray[x][y] << std::endl;
        }
        std::cout << std::endl;
    }
    */
}

void AStar::getMapArray()
{
    mapArray = new int*[mapWidth];
    mapArrayState = new int*[mapWidth];
    for(int i = 0; i < mapWidth; ++i)
    {
        mapArray[i] = new int[mapHeight];
        mapArrayState[i] = new int[mapHeight];
    }

    for (int x = 0; x < mapWidth; x++)
    {
        for (int y = 0; y < mapHeight; y++)
        {
            mapArrayState[x][y] = 0;

            int intensitycustom = (static_cast<int>(map->data[x + mapWidth * y]));
            if (intensitycustom<0 || intensitycustom>5){ 
                mapArray[x][y] = 100;
            }
            else {
                int counter2 = 0;
                int startx = x-mapTolerance;
                int starty = y-mapTolerance; 
                int endx = x+mapTolerance+1;
                int endy = y+mapTolerance+1; 
                if (startx<0){startx = 0;}
                if (starty<0){starty = 0;}
                if (endx>mapWidth){endx = mapWidth;}
                if (endy>mapHeight){endy = mapHeight;}

                for (int u = startx ; u < endx ; u++)
                {
                    for (int v = starty ; v < endy; v++)
                    {                        
                        int intensitycustom2 = (static_cast<int>(map->data[u + mapWidth * v]));
                        if (intensitycustom2<0 || intensitycustom2>5){ 
                            counter2++;
                        }
                    }
                }

                if (counter2==0){
                    mapArray[x][y] = 0;
                } else {
                    mapArray[x][y] = 100;
                }
            }            
        }
    }
}

void AStar::getPath()
{
    PointStar endpoint((int)((endPosition_x - mapOrigin_x) / mapResolution), (int)((endPosition_y - mapOrigin_y) / mapResolution));
    PointStar startpoint((int)((startPosition_x - mapOrigin_x) / mapResolution), (int)((startPosition_y - mapOrigin_y) / mapResolution));
    startpoint.gcost = 0;
    startpoint.hcost = getCost(startPosition_x, startPosition_y, endPosition_x, endPosition_y);
    startpoint.fcost = startpoint.gcost + startpoint.hcost;

    std::vector<PointStar> arropen, arrclose;
    arropen.push_back(startpoint);
    mapArrayState[startPosition_x][startPosition_y] = 1;


    PointStar pointtoadd(0.0, 0.0);
    PointStar currentpoint(0.0, 0.0);

    int buscar = 0;
    while (buscar == 0)
    {
        /** Current = to point with lower f_cost*/
        currentpoint = arropen[0];
        int posselected = 0;
        for (int i = 1; i < arropen.size(); i++)
        {
            if (arropen[i].fcost < arropen[0].fcost)
            {
                currentpoint = arropen[i];
                posselected = i;
            }
        }


        /** Add current to closed nodRosNodeAstares */
        arrclose.push_back(currentpoint);
        /** Remove current from open nodes */
        arropen.erase(arropen.begin() + posselected);
        /** Set state to closed */
        mapArrayState[currentpoint.x][currentpoint.y] = 2;

        /** Check if goal is reached*/
        if (currentpoint.x == endpoint.x && currentpoint.y == endpoint.y)
        {
            buscar = 1;
        }

        for (int i = currentpoint.x - 1; i < currentpoint.x + 2; i++)
        {
            for (int j = currentpoint.y - 1; j < currentpoint.y + 2; j++)
            {

                /** Skip same point */
                if (i == currentpoint.x && j == currentpoint.y)
                {
                    endpoint.parentx = i;
                    endpoint.parenty = j;
                    continue;
                }

                /** I is not traversable add to closed nodes and continue with the next point*/
                if (mapArray[i][j] > 10)
                {
                    pointtoadd.x = i;
                    pointtoadd.y = j;
                    pointtoadd.parentx = 999999;
                    pointtoadd.parenty = 999999;
                    pointtoadd.gcost = 999999;
                    pointtoadd.hcost = 999999;
                    pointtoadd.fcost = pointtoadd.gcost + pointtoadd.hcost;
                    arrclose.push_back(pointtoadd);
                    mapArrayState[pointtoadd.x][pointtoadd.y] = 2;
                    continue;
                }

                /** if neighbour is open check if is a shortest path */
                if (mapArrayState[i][j] == 1)
                {
                    for (int k = 0; k < arropen.size(); k++)
                    {
                        if (arropen[k].x == i && arropen[k].y == j)
                        {
                            int newgcost = currentpoint.gcost + getCost(currentpoint.x, currentpoint.y, arropen[k].x, arropen[k].y);
                            if (newgcost < arropen[k].gcost)
                            {
                                arropen[k].gcost = newgcost;
                                arropen[k].fcost = arropen[k].gcost + arropen[k].hcost;
                                arropen[k].parentx = currentpoint.x;
                                arropen[k].parenty = currentpoint.y;
                            }
                        }
                    }
                }

                /** if neighbour is not closed or open add to open */
                if (mapArrayState[i][j] == 0)
                {
                    pointtoadd.x = i;
                    pointtoadd.y = j;
                    pointtoadd.parentx = currentpoint.x;
                    pointtoadd.parenty = currentpoint.y;
                    pointtoadd.gcost = currentpoint.gcost + getCost(currentpoint.x, currentpoint.y, pointtoadd.x, pointtoadd.y);
                    pointtoadd.hcost = getCost(pointtoadd.x, pointtoadd.y, endpoint.x, endpoint.y);
                    pointtoadd.fcost = pointtoadd.gcost + pointtoadd.hcost;
                    arropen.push_back(pointtoadd);
                    mapArrayState[pointtoadd.x][pointtoadd.y] = 1;
                }
            }
        }
    }

    /** A* Output */
    pathAStar.push_back(endpoint);
    while (true)
    {
        if (pathAStar[pathAStar.size() - 1].parentx == startpoint.x && pathAStar[pathAStar.size() - 1].parenty == startpoint.y)
        {
            pathAStar.push_back(startpoint);
            break;
        }
        else
        {
            for (int i = 1; i < arrclose.size(); i++)
            {
                if (arrclose[i].x!=0 && arrclose[i].y!=0 && arrclose[i].x == pathAStar[pathAStar.size() - 1].parentx && arrclose[i].y == pathAStar[pathAStar.size() - 1].parenty)
                {
                    
                    pathAStar.push_back(arrclose[i]);
                }
            }
        }
    }
}

void AStar::getPoints(std::vector<PointStar> inputVector)
{
    pathAStarSelectedPoints.clear();
    pathAStarSelectedPoints.push_back(inputVector.front());
    float pendiente = 999999999999.0;
    for (int i= 1; i< inputVector.size(); i++){

        float hip = sqrt(pow(inputVector[i].x-pathAStarSelectedPoints[pathAStarSelectedPoints.size()-1].x,2)+pow(inputVector[i].y-pathAStarSelectedPoints[pathAStarSelectedPoints.size()-1].y,2));
        if (hip > 6){
            pathAStarSelectedPoints.push_back(inputVector[i]);
            continue;
        }
        if (abs(inputVector[i].x-pathAStarSelectedPoints[pathAStarSelectedPoints.size()-1].x)>1 && abs(inputVector[i].y-pathAStarSelectedPoints[pathAStarSelectedPoints.size()-1].y)>1)
        {
            if(inputVector[i].x-inputVector[i-1].x!=0 ){
                float pendientetmp = (inputVector[i].y-inputVector[i-1].y)/(inputVector[i].x-inputVector[i-1].x);
                if(pendiente!=pendientetmp){
                    pathAStarSelectedPoints.push_back(inputVector[i]);
                    pendiente=pendientetmp;
                }
            } else {
                float pendientetmp = 0.0;
                if(pendiente!=pendientetmp){
                    pathAStarSelectedPoints.push_back(inputVector[i]);
                    pendiente=pendientetmp;
                }
            }
        }   
    }
    pathAStarSelectedPoints.push_back(inputVector.back());
}

void AStar::getDensePath(std::vector<PointStar> inputVector)
{
    pathAStarDense.push_back(inputVector.front());

    for (int startpos = 0 ; startpos <= inputVector.size()-2 ; startpos++){
        int countertemp1 = 0;
        int countertemp2 = 0;
        for (int endpos = inputVector.size()-1 ; endpos > startpos ; endpos--){
            countertemp1++;
            int separationX = inputVector[endpos].x-inputVector[startpos].x;
            int separationY = inputVector[endpos].y-inputVector[startpos].y;
            int divisor = abs(separationX);
            if (abs(separationX)<abs(separationY)){
                divisor = abs(separationY);
            }

            double incX = (double)separationX/divisor;
            double incY = (double)separationY/divisor;

            for (int lll = 0 ; lll<abs(divisor) ; lll++){
                int mapposx= (int)(inputVector[startpos].x+(lll*(double)incX));
                int mapposy= (int)(inputVector[startpos].y+(lll*(double)incY));
                if (mapArray[mapposx][mapposy] == 100){
                    countertemp2++;
                    break;
                }
            }
            if (countertemp1!=countertemp2){
                break;
            }
        }

        
        int X1 = pathAStarDense[pathAStarDense.size()-1].x;
        int Y1 = pathAStarDense[pathAStarDense.size()-1].y;
        int X2 = inputVector[inputVector.size()-countertemp1].x;
        int Y2 = inputVector[inputVector.size()-countertemp1].y;
        int pixldistance = sqrt(pow(X2-X1,2)+pow(Y2-Y1,2));
        for (int stepPx = 0 ; stepPx < pixldistance ; stepPx++){
            PointStar pointStarTMP;
            pointStarTMP.x = ((((double)X2-X1)/pixldistance)*stepPx)+X1;
            pointStarTMP.y = ((((double)Y2-Y1)/pixldistance)*stepPx)+Y1;
            pathAStarDense.push_back(pointStarTMP);           
        }
        pathAStarDense.push_back(inputVector[inputVector.size()-countertemp1]);
        startpos = inputVector.size()-countertemp1-1;
            
    }

    int X1 = pathAStarDense[pathAStarDense.size()-1].x;
    int Y1 = pathAStarDense[pathAStarDense.size()-1].y;
    int X2 = inputVector.back().x;
    int Y2 = inputVector.back().y;
    int pixldistance = sqrt(pow(X2-X1,2)+pow(Y2-Y1,2));
    for (int stepPx = 0 ; stepPx < pixldistance ; stepPx++){
        PointStar pointStarTMP;
        pointStarTMP.x = ((((double)X2-X1)/pixldistance)*stepPx)+X1;
        pointStarTMP.y = ((((double)Y2-Y1)/pixldistance)*stepPx)+Y1;
        pathAStarDense.push_back(pointStarTMP);           
    }
    pathAStarDense.push_back(inputVector.back());  
}


void AStar::getSmoothedPath(std::vector<PointStar> inputVector)
{
    std::vector<PointStar> pathVector = inputVector;
    for (int k = 0 ; k < pathVector.size()-1 ; k++){
        if(pathVector[k].x==pathVector[k+1].x && pathVector[k].y==pathVector[k+1].y){
            pathVector.erase(pathVector.begin()+k);
        } 
    }

    std::vector<PointStar> tmpPathVector(pathVector.size());
    int stepSmooth = 10;
    int itemsSmooth =30;
    int cantIter = ceil(pathVector.size()/stepSmooth);
    for (int k = 0 ; k < cantIter ; k++){
        int startNumber = stepSmooth * k;
        int endNumber = startNumber + itemsSmooth;
        if (endNumber>pathVector.size()) {
            endNumber = pathVector.size();
            startNumber = endNumber-itemsSmooth;
        }

        for (int j = startNumber; j < endNumber ; j++)
        {
            float t = ((double)j - startNumber) / (itemsSmooth - 1);
            float x = 0.01;
            float y = 0.01;
            for (int i = startNumber; i < endNumber; i++)
            {
                unsigned long long int fact4 = smoothFormula(itemsSmooth - 1, i- startNumber);
                x += fact4 * pow(1 - (double)t, itemsSmooth - (i - startNumber)-1) * pow(t, i - startNumber) * pathVector[i].x;
            
                y += fact4 * pow(1 - (double)t, itemsSmooth - (i - startNumber)-1) * pow(t, i - startNumber) * pathVector[i].y;
            }

            tmpPathVector[j].x = x;
            tmpPathVector[j].y = y;   
        }
    }

    for (int k = 0 ; k < tmpPathVector.size()-1 ; k++){
        if(tmpPathVector[k].x==0 && tmpPathVector[k].y==0){
            pathVector.erase(pathVector.begin()+k);
        } 
    }
    pathAStarSmoothed = tmpPathVector;
}

unsigned long long int AStar::smoothFormula(int n, int i)
{
    int upper = n;
    int lower = 0;
    int denom = 0;
    if (i > n - i)
    {
        lower = i;
        denom = n - i;
    }
    else
    {
        lower = n - i;
        denom = i;
    }
    unsigned long long int nom = 1;
    for (int f = lower + 1; f <= upper; f++)
    {
        unsigned long long int p = nom;
        nom *= f;
    }
    unsigned long long int den = 1;
    for (int f = 2; f <= denom; f++)
    {
        unsigned long long int p = den;
        den *= f;
    }
    return nom / den;
}

int AStar::getCost(int startx, int starty, int endx, int endy)
{
    int getCost = 0;
    while (startx != endx || starty != endy)
    {
        if (startx != endx && starty != endy)
        {
            getCost = getCost + 14;
        }
        else
        {
            getCost = getCost + 10;
        }

        if (startx < endx)
        {
            startx++;
        }
        else if (startx > endx)
        {
            startx--;
        }

        if (starty < endy)
        {
            starty++;
        }
        else if (starty > endy)
        {
            starty--;
        }
    }
    return getCost;
}

void AStar::applyAStar(int newStartPosition_x, int newStartPosition_y, int newEndPosition_x, int newEndPosition_y)
{
    startPosition_x = newStartPosition_x;
    startPosition_y = newStartPosition_y;
    endPosition_x = newEndPosition_x;
    endPosition_y = newEndPosition_y;

    std::cout << "start position x: " << startPosition_x << std::endl;
    std::cout << "start position y: " << startPosition_y << std::endl;
    std::cout << "end position x: " << endPosition_x << std::endl;
    std::cout << "end position y: " << endPosition_y << std::endl;
    std::cout << "map width: " << mapWidth << std::endl;
    std::cout << "map heigth: " << mapHeight << std::endl;
    std::cout << "getting map array...\n";

    // DEBUG
    std::cout << "getMapArray() finished\n";

    getPath();

    // DEBUG
    std::cout << "getPath() finished\n";

    getDensePath(pathAStar);

    // DEBUG
    std::cout << "getDensePath() finished\n";

    getPoints(pathAStarDense);

    // DEBUG
    std::cout << "getPoints() finished\n";

    getSmoothedPath(pathAStarSelectedPoints);
}
