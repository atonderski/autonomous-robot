#include "behaviour-reflex.hpp"

void BehaviourReflex::initialState() noexcept {
    if (*m_pFrontDistance < REFLEX_CRITICAL_FRONT) {
        setState(&BehaviourReflex::stateMoveBack);
    }
}

void BehaviourReflex::stateMoveBack() noexcept {
    m_groundSteeringAngle = 0;
    m_pedalPosition = MIN_PEDAL_BACKWARD;
    if (*m_pFrontDistance > REFLEX_STOP_FRONT || *m_pRearDistance < REFLEX_STOP_REAR) {
        setState();
    }
}
