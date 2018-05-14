
#include "pathfinder.hpp"

bool Pathfinder::step() noexcept {
    m_stepFront = getFrontDistance();
    m_stepRear = getRearDistance();
    m_stepLeft = getLeftDistance();
    m_stepRight = getRightDistance();
    m_stepX = getX();
    m_stepY = getY();
    m_stepYaw = getYaw();
    
    m_behaviourReflex.update();
    m_behaviourAvoid.update();
    m_behaviourFollowPath.update();

    int pedalLogic;
    if (m_behaviourReflex.getActivationStatus()) {
        setGroundSteeringAngle(m_behaviourReflex.m_groundSteeringAngle);
        pedalLogic = m_behaviourReflex.m_pedalLogic;
        //std::cout << "Now in REFLEX State" << std::endl;
    } else if (m_behaviourAvoid.getActivationStatus()) {
        setGroundSteeringAngle(m_behaviourAvoid.m_groundSteeringAngle);
        pedalLogic = m_behaviourAvoid.m_pedalLogic;
        //std::cout << "Now in AVOID State" << std::endl;
    } else if (m_behaviourFollowPath.getActivationStatus()) {
        setGroundSteeringAngle(m_behaviourFollowPath.m_groundSteeringAngle);
        pedalLogic = m_behaviourFollowPath.m_pedalLogic;
        //std::cout << "Now in PATHFOLLOW State" << std::endl;
    } else {
        setGroundSteeringAngle(0);
        pedalLogic = 0;
        //std::cout << "Now in Catch-all-state State" << std::endl;
    }
    
    m_accelerationRegulator.setPedalLogic(pedalLogic);
    setPedalPositionUnscaled(m_accelerationRegulator.getPedalPosition());
    return true;
}
