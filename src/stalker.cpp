#include "stalker.hpp"

bool Stalker::step() noexcept {
    m_stepFront = getFrontDistance();
    m_stepRear = getRearDistance();
    m_stepLeft = getLeftDistance();
    m_stepRight = getRightDistance();
    m_stepDetectionAngle = getDetectionAngle();
    
    m_behaviourFollowRobot.update();

    // Subsumption goes here:
    int pedalLogic{};
    if (m_behaviourFollowRobot.getActivationStatus()) {
        setGroundSteeringAngle(m_behaviourFollowRobot.m_groundSteeringAngle);
        pedalLogic = m_behaviourFollowRobot.m_pedalLogic;
    } else {
        setGroundSteeringAngle(0);
        pedalLogic = 0;
    }

    m_accelerationRegulator.setPedalLogic(pedalLogic);
    setPedalPositionUnscaled(m_accelerationRegulator.getPedalPosition());
    return true;
}