#include "behaviour-avoid.hpp"

void BehaviourAvoid::initialState() noexcept {
    if (*m_pLeftDistance < AVOID_ACT_LEFTRIGHT) {
        setState(&BehaviourAvoid::stateTurnRight);
    } else if (*m_pRightDistance < AVOID_ACT_LEFTRIGHT) {
        setState(&BehaviourAvoid::stateTurnLeft);
    } else if (*m_pFrontDistance < AVOID_ACT_FRONT) {
        if (previousTurnRight) {
            setState(&BehaviourAvoid::stateTurnRight);
        } else {
            setState(&BehaviourAvoid::stateTurnLeft);
        }
    }
}

void BehaviourAvoid::stateTurnLeft() noexcept {
    previousTurnRight = false;
    if ((*m_pFrontDistance > AVOID_ACT_FRONT) && (m_countdown < 0)) {
        m_countdown = AVOID_TURNTIME;
        setState();
    } else {
        m_groundSteeringAngle = MAX_STEERING_LEFT;
        m_pedalLogic = 1;
        if (*m_pRightDistance < AVOID_ACT_LEFTRIGHT) {
            m_countdown = AVOID_TURNTIME;
        } else {
            m_countdown -= m_dt;
        }
    }
}

void BehaviourAvoid::stateTurnRight() noexcept {
    previousTurnRight = true;
    if ((*m_pFrontDistance > AVOID_ACT_FRONT) && (m_countdown < 0)) {
        m_countdown = AVOID_TURNTIME;
        setState();
    } else {
        m_groundSteeringAngle = MAX_STEERING_RIGHT;
        m_pedalLogic = 1;
        if (*m_pLeftDistance < AVOID_ACT_LEFTRIGHT) {
            m_countdown = AVOID_TURNTIME;
        } else {
            m_countdown -= m_dt;
        }
    }
}
