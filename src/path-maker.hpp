#ifndef PATH_MAKER
#define PATH_MAKER

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <list>
#include <vector>
#include <cmath>
#include "configurationvalues.hpp"


class PathMaker {
    public:
        PathMaker(ConfigurationValues m_conf) noexcept
        : path{}
        , startPoint{m_conf.confMap["START_POINT_X"],m_conf.confMap["START_POINT_Y"]}
        , endPoint{m_conf.confMap["END_POINT_X"],m_conf.confMap["END_POINT_Y"]}
        , MARGIN{m_conf.confMap["MARGIN"]}
        , GRID_RESOLUTION{m_conf.confMap["GRID_RESOLUTION"]}
        {
            //Read map, get list of obstacle POINTS OK!
            auto obstaclePoints = findObstaclePoints();

            //Make Grid based on largest and smallest x/y vals from obstacle list OK!
            auto gridPoints = makeGridPoints(&obstaclePoints);
            
            //Calculate points too close to obstacles -> initial banlist
            auto initialBanlist = makeInitialBanlist(&obstaclePoints, &gridPoints);
            
            // FindNearestGridPointINDEX for startPoint and endPoint
            auto startGridPointIndex = findNearestGridPointIndex(&gridPoints, &startPoint);
            auto endGridPointIndex = findNearestGridPointIndex(&gridPoints, &endPoint);

            // Ready for dijk's
            std::vector<std::pair<double,double>> gridPointsVector { std::begin(gridPoints), std::end(gridPoints) };

            /*
            std::cout << "Testing vecot!" << std::endl;
            for (auto pair : gridPointsVector) {
                std::cout << pair.first << ", " << pair.second << std::endl;
            }
            */

            auto gridPointPathList = findPath(&gridPointsVector, initialBanlist, startGridPointIndex, endGridPointIndex);
            /*
            std::cout << "Here comes the path!" << std::endl;
            for (auto pair : gridPointPathList) {
                std::cout << pair.first << ", " << pair.second << std::endl;
            }
            */

            // Smoothen that curve!
            auto smoothGridPointPathList = smoothenPath(&gridPointPathList);
            /*
            std::cout << "Here comes the smooth list" << std::endl;
            for (auto smoothGridPoint : smoothGridPointPathList) {
                std::cout << smoothGridPoint.first << ", " << smoothGridPoint.second << std::endl;
            }
            */

            //std::cout << "The start grid index is: " << startGridPointIndex << ", the end grid index is: " << endGridPointIndex << std::endl;

            path = smoothGridPointPathList;
        }

        ~PathMaker() noexcept = default;

        struct PathStruct
        {
            int pathLength;
            std::list<int> path;
            double astar;

            PathStruct()
            : pathLength{}
            , path{}
            , astar{}
            {
            }
        };

        std::list<std::pair<double,double>> path;

        std::list<std::pair<double,double>> smoothenPath(std::list<std::pair<double,double>> *) noexcept;
        std::list<std::pair<double,double>> findPath(std::vector<std::pair<double,double>> *, std::list<int>, int, int) noexcept;
        std::list<int> getNearestNeighbours(std::list<int> *, int) noexcept;
        double getManhattanDistance(std::vector<std::pair<double,double>> *, int, int) noexcept;
        int findNearestGridPointIndex(std::list<std::pair<double,double>> *, std::pair<double,double> *) noexcept;
        std::list<int> makeInitialBanlist(std::list<std::pair<double,double>> *, std::list<std::pair<double,double>> *) noexcept;
        bool isTooClose(std::list<std::pair<double,double>> *, std::pair<double,double> *) noexcept;
        std::list<std::pair<double,double>> makeGridPoints(std::list<std::pair<double,double>> *) noexcept;
        std::list<std::pair<double,double>> findObstaclePoints() noexcept;

    private:
        std::pair<double,double> startPoint;
        std::pair<double,double> endPoint;

        double const MARGIN;
        double const GRID_RESOLUTION;
};

#endif