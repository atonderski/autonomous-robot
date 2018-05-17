#include "behaviour-reflex.hpp"

void BehaviourReflex::universalState() noexcept {
    if (m_countdown >= 0) { m_countdown -= m_dt; }
}

void BehaviourReflex::initialState() noexcept {
    if (*m_pFrontDistance < REFLEX_CRITICAL_FRONT) {
        setState(&BehaviourReflex::stateMoveBack);
        m_countdown = REFLEX_STOP_MIN_TIME;
    }
}

void BehaviourReflex::stateMoveBack() noexcept {
    m_groundSteeringAngle = 0;
    m_pedalLogic = -1;
    if ((*m_pFrontDistance > REFLEX_STOP_FRONT && m_countdown < 0) || *m_pRearDistance < REFLEX_STOP_REAR || *m_pLeftDistance < REFLEX_STOP_LEFTRIGHT || *m_pRightDistance < REFLEX_STOP_LEFTRIGHT) {
        setState();
    }
}
