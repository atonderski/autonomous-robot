#include "stalker.hpp"

bool Stalker::step() noexcept {
    m_stepFront = getFrontDistance();
    m_stepRear = getRearDistance();
    m_stepLeft = getLeftDistance();
    m_stepRight = getRightDistance();

    // Behaviour.update() goes here!
    //behaviourObject1.update();

    // Subsumption goes here:
    int pedalLogic{};
    /*
    if (behaviourObject1.getActivationStatus()) {
        setGroundSteeringAngle(behaviourObject1.m_groundSteeringAngle);
        pedalLogic = behaviourObject1.m_pedalLogic;
    } else {
        setGroundSteeringAngle(0);
        pedalLogic = 0;
    }
    */

    m_accelerationRegulator.setPedalLogic(pedalLogic);
    setPedalPositionUnscaled(m_accelerationRegulator.getPedalPosition());
    return true;
}