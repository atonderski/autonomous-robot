
#include "maintainDistance.hpp"

UnlimitedStateWorks::UnlimitedStateWorks() noexcept:
    m_frontDistance{std::numeric_limits<double>::max()},
    m_rearDistance{std::numeric_limits<double>::max()},
    m_leftDistance{std::numeric_limits<double>::max()},
    m_rightDistance{std::numeric_limits<double>::max()},
    m_groundSteeringAngle{0.0},
    m_pedalPosition{0.0},
    activeState{&UnlimitedStateWorks::initialState},
    CRITICAL_DISTANCE{0.4},
    MARGIN{0.1}
{
}

void UnlimitedStateWorks::initialState() noexcept {
    std::cout << "Now in Initial State" << std::endl;
    activeState = &UnlimitedStateWorks::stateCorrectDistance;
}

void UnlimitedStateWorks::stateCloseDistance() noexcept {
    m_groundSteeringAngle = 0.0;
    m_pedalPosition = 0.7;
    std::cout << "Now in Close Distance" << std::endl;
    if (m_frontDistance < (CRITICAL_DISTANCE + MARGIN)) {
        activeState = &UnlimitedStateWorks::stateCorrectDistance;
    }
}

void UnlimitedStateWorks::stateMakeDistance() noexcept {
    m_groundSteeringAngle = 0.0;
    m_pedalPosition = -1.0;
    std::cout << "Now in Make Distance" << std::endl;
    if (m_frontDistance > (CRITICAL_DISTANCE - MARGIN)) {
        activeState = &UnlimitedStateWorks::stateCorrectDistance;
    }
}

void UnlimitedStateWorks::stateCorrectDistance() noexcept {
    m_groundSteeringAngle = 0.0;
    m_pedalPosition = 0.0;
    std::cout << "Now in Correct Distance" << std::endl;
    if (m_frontDistance < (CRITICAL_DISTANCE - MARGIN)) {
        activeState = &UnlimitedStateWorks::stateMakeDistance;
    } else if (m_frontDistance > (CRITICAL_DISTANCE + MARGIN)) {
        activeState = &UnlimitedStateWorks::stateCloseDistance;
    } else {
        activeState = &UnlimitedStateWorks::stateCorrectDistance;
    }
}

void UnlimitedStateWorks::update() noexcept
{
    if (activeState != 0) {
        (this->*activeState)();
    }
}

bool MaintainDistance::step(double) noexcept {
    m_satanicMill.m_frontDistance = getFrontDistance();
    //m_satanicMill.m_rearDistance = getRearDistance();
    //m_satanicMill.m_leftDistance = getLeftDistance();
    //m_satanicMill.m_rightDistance = getRightDistance();

    m_satanicMill.update();

    setGrounSteeringAngle(m_satanicMill.m_groundSteeringAngle);
    setPedalPosition(m_satanicMill.m_pedalPosition);
    
    return true;
}
