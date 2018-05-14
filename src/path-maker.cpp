#include "path-maker.hpp"

int PathMaker::findNearestGridPointIndex(std::list<std::pair<double,double>> *ptrGridPoints, std::pair<double,double> *ptrPoint) noexcept {
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

std::list<int> PathMaker::makeInitialBanlist(std::list<std::pair<double,double>> *ptrObstaclePoints, std::list<std::pair<double,double>> *ptrGridPoints) noexcept {
    std::list<int> initialBanlist{};
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

bool PathMaker::isTooClose(std::list<std::pair<double,double>> *ptrObstaclePoints, std::pair<double,double> *ptrGridPoint) noexcept {
    for (auto obstaclePoint : *ptrObstaclePoints) {
        if (std::sqrt(std::pow((obstaclePoint.first - ptrGridPoint->first), 2) + std::pow((obstaclePoint.second - ptrGridPoint->second), 2)) < MARGIN) {
            return true;
        }
    }
    return false;
}

std::list<std::pair<double,double>> PathMaker::makeGridPoints(std::list<std::pair<double,double>> *ptrObstaclePoints) noexcept {
    double minX{100};
    double maxX{-100};
    double minY{100};
    double maxY{-100};
    for (auto p : *ptrObstaclePoints) {
        if (p.first < minX) {
            minX = p.first;
        }
        if (p.first > maxX) {
            maxX = p.first;
        }
        if (p.second < minY) {
            minY = p.second;
        }
        if (p.second > maxY) {
            maxY = p.second;
        }
    }
    //std::cout << minX << ", " << maxX << ", " << minY << ", " << maxY << std::endl;

    std::list<double> xRange{};
    std::list<double> yRange{};
    double diffX = (maxX - minX)/(GRID_RESOLUTION - 1);
    double diffY = (maxY - minY)/(GRID_RESOLUTION - 1);
    double X = minX;
    xRange.push_back(X);
    for (int i = 1; i < GRID_RESOLUTION; ++i) {
        X += diffX;
        xRange.push_back(X);
    }
    double Y = minY;
    yRange.push_back(Y);
    for (int i = 1; i < GRID_RESOLUTION; ++i) {
        Y += diffY;
        yRange.push_back(Y);
    }
    /*
    std::cout << "X RANGE STARTS HERE" << std::endl;
    for (auto num : xRange) {
        std::cout << num << std::endl;
    }
    std::cout << "Y RANGE STARTS HERE" << std::endl;
    for (auto num : yRange) {
        std::cout << num << std::endl;
    }
    */

    std::list<std::pair<double,double>> ptrGridPoints{};    
    for (auto gridY : yRange) {
        for (auto gridX : xRange) {
            ptrGridPoints.push_back(std::make_pair(gridX, gridY));
        }
    }
    /*
    std::cout << "GRID STARTS HERE" << std::endl;
    for (auto p : ptrGridPoints) {
        std::cout << p.first << ", " << p.second << std::endl;
    }
    */

    return ptrGridPoints;
}

std::list<std::pair<double,double>> PathMaker::findObstaclePoints() noexcept {
    //Read simulation-map.txt into list of points
    std::list<std::pair<double,double>> obstaclesList{};
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
        //std::cout << startXString << startYString << endXString << endYString << std::endl;

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
    /*
    for (auto elem : obstaclesList) {
                std::cout << elem.first << " " << elem.second << std::endl;
            }
    */
}
