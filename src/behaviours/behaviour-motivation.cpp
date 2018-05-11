#include "behaviour-motivation.hpp"

void BehaviourMotivation::initialState() noexcept {
    if (*m_pFrontDistance > MOTIVATION_ACT_FRONT) {
        //setState(&BehaviourMotivation::stateMotivated);
        setState(&BehaviourMotivation::stateScan);
    }
}

void BehaviourMotivation::stateMotivated() noexcept {
    m_groundSteeringAngle = 0;
    m_pedalLogic = 1;
    if (*m_pFrontDistance < MOTIVATION_ACT_FRONT) {
        setState();
    }
}

void BehaviourMotivation::stateScan() noexcept {
    if (m_countdown <= 0) {
        if (m_groundSteeringAngle >= SCAN_STEERING_LEFT) { //>= just to get around compilation error. Yikes!
            m_groundSteeringAngle = SCAN_STEERING_RIGHT;
            m_pedalLogic = 1;
        } else {
            m_groundSteeringAngle = SCAN_STEERING_LEFT;
            m_pedalLogic = 1;
        }
        m_countdown = MOTIVATION_SCAN_SEMIPERIOD;
    } else {
        m_countdown -= m_dt;
    }
    
    if (*m_pFrontDistance < MOTIVATION_ACT_FRONT) {
        setState();
    }
}