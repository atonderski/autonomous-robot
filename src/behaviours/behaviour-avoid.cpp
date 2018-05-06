#include "behaviour-avoid.hpp"

void BehaviourAvoid::initialState() noexcept {
    if (*m_pFrontDistance < AVOID_ACT_FRONT) {
        setState(&BehaviourAvoid::stateTurnLeft);
    } else if (*m_pLeftDistance < AVOID_ACT_LEFTRIGHT) {
        setState(&BehaviourAvoid::stateTurnRight);
    } else if (*m_pRightDistance < AVOID_ACT_LEFTRIGHT) {
        setState(&BehaviourAvoid::stateTurnLeft);
    }
}

void BehaviourAvoid::stateTurnLeft() noexcept {
    if ((*m_pRearDistance < AVOID_STOP_REAR) && (*m_pFrontDistance > AVOID_ACT_FRONT)) {
        setState();
    } else {
        m_groundSteeringAngle = MAX_STEERING_LEFT;
        m_pedalPosition = MIN_PEDAL_FORWARD;
    }
}

void BehaviourAvoid::stateTurnRight() noexcept {
    if ((*m_pRearDistance < AVOID_STOP_REAR) && (*m_pFrontDistance > AVOID_ACT_FRONT)) {
        setState();
    } else {
        m_groundSteeringAngle = MAX_STEERING_RIGHT;
        m_pedalPosition = MIN_PEDAL_FORWARD;
    }
}
