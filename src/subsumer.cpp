
#include "subsumer.hpp"

bool Subsumer::step() noexcept {
    m_stepFront = getFrontDistance();
    m_stepRear = getRearDistance();
    m_stepLeft = getLeftDistance();
    m_stepRight = getRightDistance();
    
    m_behaviourReflex.update();
    m_behaviourAvoid.update();
    m_behaviourMotivation.update();

    int pedalLogic;
    if (m_behaviourReflex.getActivationStatus()) {
        setGroundSteeringAngle(m_behaviourReflex.m_groundSteeringAngle);
        pedalLogic = m_behaviourReflex.m_pedalLogic;
    } else if (m_behaviourAvoid.getActivationStatus()) {
        setGroundSteeringAngle(m_behaviourAvoid.m_groundSteeringAngle);
        pedalLogic = m_behaviourAvoid.m_pedalLogic;
    } else if (m_behaviourMotivation.getActivationStatus()) {
        setGroundSteeringAngle(m_behaviourMotivation.m_groundSteeringAngle);
        pedalLogic = m_behaviourMotivation.m_pedalLogic;
    } else {
        setGroundSteeringAngle(0);
        pedalLogic = 0;
    }
    
    m_accelerationRegulator.setPedalLogic(pedalLogic);
    setPedalPositionUnscaled(m_accelerationRegulator.getPedalPosition());
    return true;
}
