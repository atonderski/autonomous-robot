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

using Point = std::pair<double,double>;
using PointList = std::list<Point>;
using PointVector = std::vector<Point>;
using IndexList = std::list<int>;

class PathMaker {
    public:
        PathMaker(ConfigurationValues m_conf) noexcept
        : path{}
        , startPoint{m_conf.confMap["START_POINT_X"],m_conf.confMap["START_POINT_Y"]}
        , endPoint{m_conf.confMap["END_POINT_X"],m_conf.confMap["END_POINT_Y"]}
        , MARGIN{m_conf.confMap["MARGIN"]}
        , GRID_RESOLUTION{m_conf.confMap["GRID_RESOLUTION"]}
        {
            auto obstaclePoints = findObstaclePoints();
            auto gridPoints = makeGridPoints(&obstaclePoints);
            auto initialBanlist = makeInitialBanlist(&obstaclePoints, &gridPoints);
            auto startGridPointIndex = findNearestGridPointIndex(&gridPoints, &startPoint);
            auto endGridPointIndex = findNearestGridPointIndex(&gridPoints, &endPoint);

            PointVector gridPointsVector { std::begin(gridPoints), std::end(gridPoints) };

            auto gridPointPathList = findPath(&gridPointsVector, initialBanlist, startGridPointIndex, endGridPointIndex);
            auto smoothGridPointPathList = smoothenPath(&gridPointPathList);
            path = smoothGridPointPathList;
        }

        ~PathMaker() noexcept = default;

        struct PathStruct
        {
            int pathLength;
            IndexList path;
            double astar;

            PathStruct()
            : pathLength{}
            , path{}
            , astar{}
            {
            }
        };

        PointList path;

        PointList smoothenPath(PointList *) noexcept;
        PointList findPath(PointVector *, IndexList, int, int) noexcept;
        IndexList getNearestNeighbours(IndexList *, int) noexcept;
        double getManhattanDistance(PointVector *, int, int) noexcept;
        int findNearestGridPointIndex(PointList *, Point *) noexcept;
        IndexList makeInitialBanlist(PointList *, PointList *) noexcept;
        bool isTooClose(PointList *, Point *) noexcept;
        PointList makeGridPoints(PointList *) noexcept;
        PointList findObstaclePoints() noexcept;

    private:
        Point startPoint;
        Point endPoint;

        double const MARGIN;
        double const GRID_RESOLUTION;
};

#endif