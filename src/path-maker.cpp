#include "path-maker.hpp"

PointList PathMaker::smoothenPath(PointList *gridPointPathList) noexcept {
    PointList smoothPathWidthOne{};
    PointList::iterator itr = gridPointPathList->begin();
    smoothPathWidthOne.push_back(*itr);
    std::advance(itr,1);
    PointList::iterator endItr = gridPointPathList->end();
    std::advance(endItr,-1);
    while (itr != endItr) {
        
        auto itrSub1 = itr;
        std::advance(itrSub1,-1);
        auto itrAdd1 = itr;
        std::advance(itrAdd1,1);
        
        double smoothX = (itrSub1->first + itr->first + itrAdd1->first)/3.0;
        double smoothY = (itrSub1->second + itr->second + itrAdd1->second)/3.0;
        smoothPathWidthOne.push_back(std::make_pair(smoothX, smoothY));
        itr++;
    }
    smoothPathWidthOne.push_back(*itr);
    
    PointList smoothPathWidthTwo{};
    itr = smoothPathWidthOne.begin();
    smoothPathWidthTwo.push_back(*(itr++));
    smoothPathWidthTwo.push_back(*(itr++));
    endItr = smoothPathWidthOne.end();
    std::advance(endItr,-2);
    while (itr != endItr) {
        
        auto itrSub2 = itr;
        std::advance(itrSub2,-2);
        auto itrSub1 = itr;
        std::advance(itrSub1,-1);
        auto itrAdd1 = itr;
        std::advance(itrAdd1,1);
        auto itrAdd2 = itr;
        std::advance(itrAdd2,2);
        
        double smoothX = (itrSub2->first + itrSub1->first + itr->first + itrAdd1->first + itrAdd2->first)/5.0;
        double smoothY = (itrSub2->second + itrSub1->second + itr->second + itrAdd1->second + itrAdd2->second)/5.0;
        smoothPathWidthTwo.push_back(std::make_pair(smoothX, smoothY));
        itr++;
    }
    smoothPathWidthTwo.push_back(*(itr++));
    smoothPathWidthTwo.push_back(*itr);

    return smoothPathWidthTwo;
}

PointList PathMaker::findPath(PointVector *ptrGridPointsVector, IndexList initialBanlist, int startGridPointIndex, int endGridPointIndex) noexcept {
    auto banlist = initialBanlist;
    std::list<PathMaker::PathStruct> pathsList{};

    PathStruct initialPathStruct;
    initialPathStruct.pathLength = 0;
    initialPathStruct.path.push_back(startGridPointIndex);
    initialPathStruct.astar = initialPathStruct.pathLength + getManhattanDistance(ptrGridPointsVector, startGridPointIndex, endGridPointIndex);
    /*
    std::cout << "Struct: pathLength = " << initialPathStruct.pathLength
              << ", path 0th elem = " << initialPathStruct.path.front()
              << ", astar = " << initialPathStruct.astar << std::endl;
    */

    pathsList.push_front(initialPathStruct);
    banlist.push_front(startGridPointIndex);
    /*
    for (auto ind : banlist) {
        std::cout << ind << std::endl;
    }
    */
    
    PointList gridPointPath{};
    bool foundFinalPath{false};
    while (!foundFinalPath) {
        //find and EXTRACT struct of lowest astar value. Copy into a separate variable
        //get index of lowest astar //copy into separate variable
        PathStruct shortestPathStruct{};
        double lowestAstar{1000};
        std::list<PathMaker::PathStruct>::iterator iteratorToShortestPathStruct;
        std::list<PathMaker::PathStruct>::iterator itr = pathsList.begin();
        while (itr != pathsList.end()) {
            if ((*itr).astar < lowestAstar) {
                iteratorToShortestPathStruct = itr;
                lowestAstar = (*itr).astar;
            }
            itr++;
        }
        shortestPathStruct = *iteratorToShortestPathStruct;
        //std::cout << "Size before erasure: " << pathsList.size() << std::endl;
        pathsList.erase(iteratorToShortestPathStruct);
        //std::cout << "Size after erasure: " << pathsList.size() << std::endl;
        //std::cout << "Current struct is = " << shortestPathStruct.pathLength << ", " << shortestPathStruct.path.front() << ", " << shortestPathStruct.astar << std::endl;

        // Find nearest legit neighbours
        IndexList neighboursList = getNearestNeighbours(&banlist, shortestPathStruct.path.back());
        /*
        std::cout << "All the neighbours... " << std::endl;
        for (auto neighbour : neighboursList) {
            std::cout << neighbour << std::endl;
        }
        */

        // add neighbours to banlist
        for (auto neighbourIndex : neighboursList) {
            banlist.push_front(neighbourIndex);
        }
        /*
        for (auto ind : banlist) {
        std::cout << ind << std::endl;
        }
        */
        
        // for all neighbours, combine with shortestPathStruct to make new structs
        for (auto neighbourIndex : neighboursList) {
            PathStruct newPathStruct = shortestPathStruct;
            newPathStruct.pathLength += 1;
            newPathStruct.path.push_back(neighbourIndex);
            newPathStruct.astar = newPathStruct.pathLength + getManhattanDistance(ptrGridPointsVector, neighbourIndex, endGridPointIndex);

            // If current new pointIndex == endGridPointIndex then return this path!
            if (neighbourIndex == endGridPointIndex) {
                // Construct grid path from index path
                for (auto gridPointIndex : newPathStruct.path) {
                    gridPointPath.push_back((*ptrGridPointsVector)[gridPointIndex]);
                }
                foundFinalPath = true;
                break;
                /*
                std::cout << "Here comes the path!" << std::endl;
                for (auto elem : newPathStruct.path) {
                    std::cout << elem << std::endl;
                }
                */
            } else { // else push struct to pathsList and keep iterating
                pathsList.push_front(newPathStruct);
            }
        }
    }

    return gridPointPath;
}

IndexList PathMaker::getNearestNeighbours(IndexList *ptrBanlist, int currentIndex) noexcept {
    IndexList neighboursList{};
    double maxGridIndex = GRID_RESOLUTION*GRID_RESOLUTION;
    int gridRes = static_cast<int>(GRID_RESOLUTION);
    if ((currentIndex - 1) >= 0) {
        neighboursList.push_back((currentIndex - 1));
    }
    if ((currentIndex + 1) < maxGridIndex) {
        neighboursList.push_back((currentIndex + 1));
    }
    if ((currentIndex - gridRes) >= 0) {
        neighboursList.push_back(currentIndex - gridRes);
    }
    if ((currentIndex + gridRes) < maxGridIndex) {
        neighboursList.push_back(currentIndex + gridRes);
    }
    IndexList::iterator itr = neighboursList.begin();
    while (itr != neighboursList.end()) {
        bool shouldRemove{false};
        for (auto bannedIndex : *ptrBanlist) {
            if (*itr == bannedIndex) {
                shouldRemove = true;
                break;
            }
        }
        if (shouldRemove) {
            neighboursList.erase(itr++);
        } else {
            ++itr;
        }
    }

    return neighboursList;
}

double PathMaker::getManhattanDistance(PointVector *ptrGridPointsVector, int firstPointIndex, int secondPointIndex) noexcept {
    double x1 = (*ptrGridPointsVector)[firstPointIndex].first;
    double y1 = (*ptrGridPointsVector)[firstPointIndex].second;
    double x2 = (*ptrGridPointsVector)[secondPointIndex].first;
    double y2 = (*ptrGridPointsVector)[secondPointIndex].second;
    return std::abs(x2 - x1) + std::abs(y2 - y1);
}

int PathMaker::findNearestGridPointIndex(PointList *ptrGridPoints, Point *ptrPoint) noexcept {
    double shortestDistance{100};
    int nearestGridPointIndex{};
    int index{0}; // START FROM ZEROETH IN C++ IN CONTRAST TO MATLAB
    for (auto gridPoint : *ptrGridPoints) {
        double currentDistance = std::sqrt(std::pow((gridPoint.first - ptrPoint->first), 2) + std::pow((gridPoint.second - ptrPoint->second), 2));
        if (currentDistance < shortestDistance) {
            shortestDistance = currentDistance;
            nearestGridPointIndex = index;
        }
        ++index;
    }

    return nearestGridPointIndex;
}

IndexList PathMaker::makeInitialBanlist(PointList *ptrObstaclePoints, PointList *ptrGridPoints) noexcept {
    IndexList initialBanlist{};
    int gridIndex = 0; // START FROM ZEROETH IN C++ IN CONTRAST TO MATLAB
    for (auto gridPoint : *ptrGridPoints) {
        if (isTooClose(ptrObstaclePoints, &gridPoint)) {
            initialBanlist.push_back(gridIndex);
        }
        ++gridIndex;
    }
    /*
    for (auto ind : initialBanlist) {
        std::cout << ind << std::endl;
    }
    */

    return initialBanlist;
}

bool PathMaker::isTooClose(PointList *ptrObstaclePoints, Point *ptrGridPoint) noexcept {
    for (auto obstaclePoint : *ptrObstaclePoints) {
        if (std::sqrt(std::pow((obstaclePoint.first - ptrGridPoint->first), 2) + std::pow((obstaclePoint.second - ptrGridPoint->second), 2)) < MARGIN) {
            return true;
        }
    }
    return false;
}

PointList PathMaker::makeGridPoints(PointList *ptrObstaclePoints) noexcept {
    double minX{100};
    double maxX{-100};
    double minY{100};
    double maxY{-100};
    for (auto p : *ptrObstaclePoints) {
        if (p.first < minX) { minX = p.first; }
        if (p.first > maxX) { maxX = p.first; }
        if (p.second < minY) { minY = p.second; }
        if (p.second > maxY) { maxY = p.second; }
    }

    std::list<double> xRange{};
    std::list<double> yRange{};
    double const diffX{(maxX - minX)/(GRID_RESOLUTION - 1)};
    double const diffY{(maxY - minY)/(GRID_RESOLUTION - 1)};
    double X = minX;
    double Y = minY;
    xRange.push_back(X);
    yRange.push_back(Y);
    for (int i = 1; i < GRID_RESOLUTION; ++i) {
        X += diffX;
        xRange.push_back(X);
    }
    for (int i = 1; i < GRID_RESOLUTION; ++i) {
        Y += diffY;
        yRange.push_back(Y);
    }

    PointList ptrGridPoints{};    
    for (auto gridY : yRange) {
        for (auto gridX : xRange) {
            ptrGridPoints.push_back(std::make_pair(gridX, gridY));
        }
    }

    return ptrGridPoints;
}

PointList PathMaker::findObstaclePoints() noexcept {
    PointList obstaclesList{};
    std::ifstream fileInput("/opt/simulation-map.txt");
    std::string line;

    while (std::getline(fileInput, line)) {
        std::istringstream isLine(line);
        std::string startXString;
        std::string startYString;
        std::string endXString;
        std::string endYString;
        std::getline(isLine, startXString, ',');
        std::getline(isLine, startYString, ',');
        std::getline(isLine, endXString, ',');
        std::getline(isLine, endYString, ';');

        double startX = std::stod(startXString);
        double startY = std::stod(startYString);
        double endX = std::stod(endXString);
        double endY = std::stod(endYString);
        double diffX = endX - startX;
        double diffY = endY - startY;
        double L = std::sqrt( std::pow(diffX, 2) + std::pow(diffY, 2) );
        double d = 2 * std::ceil(L/(std::sqrt(3) * MARGIN));
        double dX = diffX/(d-1);
        double dY = diffY/(d-1);

        double X = startX;
        double Y = startY;
        obstaclesList.push_back(std::make_pair(X,Y));
        for (int i = 0; i < (d-1) ; ++i) {
            X += dX;
            Y += dY;
            obstaclesList.push_back(std::make_pair(X,Y));
        }
    }

    return obstaclesList;
}
