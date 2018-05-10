#include "behaviour-reflex.hpp"

void BehaviourReflex::initialState() noexcept {
    if (*m_pFrontDistance < REFLEX_CRITICAL_FRONT) {
        setState(&BehaviourReflex::stateMoveBack);
    }
}

void BehaviourReflex::stateMoveBack() noexcept {
    m_groundSteeringAngle = 0;
    m_pedalLogic = MIN_PEDAL_BACKWARD;
    if (*m_pFrontDistance > REFLEX_STOP_FRONT || *m_pRearDistance < REFLEX_STOP_REAR || *m_pLeftDistance < REFLEX_STOP_LEFTRIGHT || *m_pRightDistance < REFLEX_STOP_LEFTRIGHT) {
        setState();
    }
}
