//
// Created by Adam Tonderski on 2018-04-16.
//

#include "manualcontroller.hpp"

bool ManualController::step() noexcept {
    
    if (m_printSensorValues) {
        if (m_delayTimer > 0.5) {
            printSensorValues();
            m_delayTimer = 0;
        } else {
            m_delayTimer += m_dt;
        }
    }

    if (m_currentTime < m_timeToRun) {
        //setPedalPosition(m_manualPedalPosition);
        setPedalPositionUnscaled(m_manualPedalPosition);
        setGroundSteeringAngle(m_manualSteeringAngle);
        m_currentTime += m_dt;
        return true;
    } else {
        setPedalPosition(0.0);
        setGroundSteeringAngle(0.0);
        return false;
    }
}

void ManualController::printSensorValues() noexcept {
    double front{getFrontDistance()};
    double rear{getRearDistance()};
    double left{getLeftDistance()};
    double right{getRightDistance()};

    std::cout   << std::fixed << std::setprecision(3)
                << "Front: " << front
                << "\tRear: " << rear
                << "\tLeft: " << left
                << "\tRight: " << right
                << std::endl;
}