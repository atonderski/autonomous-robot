#include "behaviour-followpath.hpp"

void BehaviourFollowPath::universalState() noexcept {
    if (m_countdown >= 0) { m_countdown -= m_dt; }
    if (distanceToPathpoint(m_finalPathIndex) < m_stopRadius) { --finishCounter; }
}

void BehaviourFollowPath::initialState() noexcept {
    m_groundSteeringAngle = 0;
    m_pedalLogic = 0;
    if (distanceToPathpoint(m_finalPathIndex) > m_stopRadius && notFinished) {
        if (m_countdown < 0) {
            setState(&BehaviourFollowPath::followPathState);
        }
    }
}

void BehaviourFollowPath::followPathState() noexcept {
    m_groundSteeringAngle = getSteeringSignal();
    m_pedalLogic = 1;
    if (finishCounter < 0) {
        notFinished = false;
        setState();
    }
}

double BehaviourFollowPath::getSteeringSignal() noexcept {
    std::pair<double,double> previewPoint = getPreviewPoint();

    //VECTOR ALGEBRA FOR GREAT JUSTICE
    double Hx = std::cos(*m_pYaw);
    double Hy = std::sin(*m_pYaw);
    double Jx = previewPoint.first - *m_pX;
    double Jy = previewPoint.second - *m_pY;
    double HxJ = Hx*Jy - Hy*Jx;

    double magH = std::sqrt( std::pow(Hx, 2) + std::pow(Hy, 2) );
    double magJ = std::sqrt( std::pow(Jx, 2) + std::pow(Jy, 2) );
    double turnSignal = std::asin(HxJ/(magH * magJ));
    double const piHalf{1.570796};
    if (std::abs(turnSignal) <= piHalf) {
        return turnSignal*(MAX_STEERING_LEFT/piHalf);
    } else if (turnSignal > piHalf) {
        return MAX_STEERING_LEFT;
    } else {
        return MAX_STEERING_RIGHT;
    }
}

std::pair<double,double> BehaviourFollowPath::getPreviewPoint() noexcept {

    int currentClosestPathpoint{};
    double currentClosestDistance = 100;
    double tmpDist;
    for (int i = 0; i <= m_finalPathIndex; ++i) {
        tmpDist = distanceToPathpoint(i);
        if (tmpDist <= currentClosestDistance) {
            currentClosestPathpoint = i;
            currentClosestDistance = tmpDist;
        }
    }
    
    int previewPoint{currentClosestPathpoint};
    double pathSegmentLengthToPreviewPoint{0.0};
    while (pathSegmentLengthToPreviewPoint < m_previewPointOffset) {
        ++previewPoint;
        double xSegment = m_path[previewPoint].first - m_path[previewPoint - 1].first;
        double ySegment = m_path[previewPoint].second - m_path[previewPoint - 1].second;
        pathSegmentLengthToPreviewPoint += std::sqrt( std::pow(xSegment, 2) + std::pow(ySegment, 2) );
    }

    if (previewPoint > m_finalPathIndex) {
        return m_path[m_finalPathIndex];
    } else {
        return m_path[previewPoint];
    }
}

double BehaviourFollowPath::distanceToPathpoint(int iPathpoint) noexcept {
    double xdiff = std::abs(m_path[iPathpoint].first - *m_pX);
    double ydiff = std::abs(m_path[iPathpoint].second - *m_pY);
    return std::sqrt( std::pow(xdiff, 2) + std::pow(ydiff, 2) );
}

int BehaviourFollowPath::signum(double val) noexcept {
    if (val > 0) {
        return 1;
    } else if (val < 0) {
        return -1;
    } else {
        return 0;
    }
}