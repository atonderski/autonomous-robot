
#include "maintainDistance.hpp"

MaintainDistanceStates::MaintainDistanceStates() noexcept:
    CRITICAL_DISTANCE{0.4},
    MARGIN{0.1}
{
    setState(&MaintainDistanceStates::initialState);
}

void MaintainDistanceStates::initialState() noexcept {
    std::cout << "Now in Initial State" << std::endl;
    setState(&MaintainDistanceStates::stateStationary);
}

void MaintainDistanceStates::stateMoveTowards() noexcept {
    m_groundSteeringAngle = 0.0;
    m_pedalPosition = 0.7;
    std::cout << "Now moving towards" << std::endl;
    if (m_frontDistance < (CRITICAL_DISTANCE + MARGIN)) {
        setState(&MaintainDistanceStates::stateStationary);
    }
}

void MaintainDistanceStates::stateMoveAway() noexcept {
    m_groundSteeringAngle = 0.0;
    m_pedalPosition = -1.0;
    std::cout << "Now moving away" << std::endl;
    if (m_frontDistance > (CRITICAL_DISTANCE - MARGIN)) {
        setState(&MaintainDistanceStates::stateStationary);
    }
}

void MaintainDistanceStates::stateStationary() noexcept {
    m_groundSteeringAngle = 0.0;
    m_pedalPosition = 0.0;
    std::cout << "Now standing still" << std::endl;
    if (m_frontDistance < (CRITICAL_DISTANCE - MARGIN)) {
        setState(&MaintainDistanceStates::stateMoveAway);
    } else if (m_frontDistance > (CRITICAL_DISTANCE + MARGIN)) {
        setState(&MaintainDistanceStates::stateMoveTowards);
    } 
}

bool MaintainDistance::step(double) noexcept {
    m_stateMachine.m_frontDistance = getFrontDistance();
    //m_stateMachine.m_rearDistance = getRearDistance();
    //m_stateMachine.m_leftDistance = getLeftDistance();
    //m_stateMachine.m_rightDistance = getRightDistance();

    m_stateMachine.update();

    setGrounSteeringAngle(m_stateMachine.m_groundSteeringAngle);
    setPedalPosition(m_stateMachine.m_pedalPosition);
    
    return true;
}
