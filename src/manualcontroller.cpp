//
// Created by Adam Tonderski on 2018-04-16.
//

#include "manualcontroller.hpp"

ManualController::ManualController(double pedalPosition, double steeringAngle, double timeToRun) noexcept:
    m_manualPedalPosition{pedalPosition},
    m_manualSteeringAngle{steeringAngle},
    m_timeToRun{timeToRun},
    m_currentTime{0.0}
{
}

bool ManualController::step(double dt) noexcept {
    if (m_currentTime < m_timeToRun) {
        setPedalPosition(m_manualPedalPosition);
        setGroundSteeringAngle(m_manualSteeringAngle);
        m_currentTime += dt;
        return true;
    } else {
        setPedalPosition(0.0);
        setGroundSteeringAngle(0.0);
        return false;
    }
}
