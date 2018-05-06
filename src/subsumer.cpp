
#include "subsumer.hpp"

bool Subsumer::step() noexcept {
    m_stepFront = getFrontDistance();
    m_stepRear = getRearDistance();
    m_stepLeft = getLeftDistance();
    m_stepRight = getRightDistance();
    
    m_behaviourReflex.update();
    m_behaviourAvoid.update();
    m_behaviourMotivation.update();

    if (m_behaviourReflex.getActivationStatus()) {
        setGroundSteeringAngle(m_behaviourReflex.m_groundSteeringAngle);
        setPedalPosition(m_behaviourReflex.m_pedalPosition);
    } else if (m_behaviourAvoid.getActivationStatus()) {
        setGroundSteeringAngle(m_behaviourAvoid.m_groundSteeringAngle);
        setPedalPosition(m_behaviourAvoid.m_pedalPosition);
    } else if (m_behaviourMotivation.getActivationStatus()) {
        setGroundSteeringAngle(m_behaviourMotivation.m_groundSteeringAngle);
        setPedalPosition(m_behaviourMotivation.m_pedalPosition);
    } else {
        setGroundSteeringAngle(0);
        setPedalPosition(0);
    }
    
    
    return true;
}
