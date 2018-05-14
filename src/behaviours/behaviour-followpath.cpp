#include "behaviour-followpath.hpp"

void BehaviourFollowPath::initialState() noexcept {
    m_groundSteeringAngle = 0;
    m_pedalLogic = 0;
    if (distanceToPathpoint(m_finalPathIndex) > 0.2) {
        setState(&BehaviourFollowPath::FollowPathState);
    }
}

void BehaviourFollowPath::FollowPathState() noexcept {
    m_groundSteeringAngle = getSteeringSignal();
    m_pedalLogic = 1;
    if (distanceToPathpoint(m_finalPathIndex) < 0.2) {
        setState();
    }
}

double BehaviourFollowPath::getSteeringSignal() noexcept {
    std::pair<double,double> previewPoint = getPreviewPoint();

    //VECTOR ALGEBRA FOR GREAT JUSTICE
    double Px = previewPoint.first;
    double Py = previewPoint.second;
    double Cx = *m_pX;
    double Cy = *m_pY;
    double phi = *m_pYaw;
    double Hx = std::cos(phi);
    double Hy = std::sin(phi);
    double Jx = Px - Cx;
    double Jy = Py - Cy;
    double HxJ = Hx*Jy - Hy*Jx;

    double magH = std::sqrt( std::pow(Hx, 2) + std::pow(Hy, 2) );
    double magJ = std::sqrt( std::pow(Jx, 2) + std::pow(Jy, 2) );
    double turnSignal = std::asin(HxJ/(magH * magJ));
    double piHalf = 1.570796;
    if (std::abs(turnSignal) <= piHalf) {
        return turnSignal*(MAX_STEERING_LEFT/piHalf);
    } else if (turnSignal > piHalf) {
        return MAX_STEERING_LEFT;
    } else {
        return MAX_STEERING_RIGHT;
    }

    //return signum(HxJ) * 0.30;
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
    
    int previewPoint = currentClosestPathpoint + m_previewPointOffset;
    //std::cout << "Current preview point = " << previewPoint << std::endl;
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