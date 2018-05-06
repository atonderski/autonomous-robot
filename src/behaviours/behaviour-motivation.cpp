#include "behaviour-motivation.hpp"

void BehaviourMotivation::initialState() noexcept {
    if (*m_pFrontDistance > MOTIVATION_ACT_FRONT) {
        setState(&BehaviourMotivation::stateMotivated);
    }
}

void BehaviourMotivation::stateMotivated() noexcept {
    m_groundSteeringAngle = 0;
    m_pedalPosition = MIN_PEDAL_FORWARD;
    if (*m_pFrontDistance < MOTIVATION_ACT_FRONT) {
        setState();
    }
}
