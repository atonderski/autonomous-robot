#include "behaviour-avoid.hpp"

void BehaviourAvoid::initialState() noexcept {
    if (*m_pLeftDistance < AVOID_ACT_LEFTRIGHT) {
        avoidingFront = false;
        m_countdown = AVOID_TURNTIME_LEFTRIGHT;
        setState(&BehaviourAvoid::stateTurnRight);
    } else if (*m_pRightDistance < AVOID_ACT_LEFTRIGHT) {
        avoidingFront = false;
        m_countdown = AVOID_TURNTIME_LEFTRIGHT;
        setState(&BehaviourAvoid::stateTurnLeft);
    } else if (*m_pFrontDistance < AVOID_ACT_FRONT) {
        avoidingFront = true;
        m_countdown = AVOID_TURNTIME_FRONT;
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
        setState();
    } else {
        m_groundSteeringAngle = MAX_STEERING_LEFT;
        m_pedalLogic = 1;
        if (*m_pRightDistance < AVOID_ACT_LEFTRIGHT) {
            m_countdown = AVOID_TURNTIME_LEFTRIGHT;
        } else {
            m_groundSteeringAngle = MAX_STEERING_LEFT/2;
            m_countdown -= m_dt;
        }
    }
}

void BehaviourAvoid::stateTurnRight() noexcept {
    previousTurnRight = true;
    if ((*m_pFrontDistance > AVOID_ACT_FRONT) && (m_countdown < 0)) {
        setState();
    } else {
        m_groundSteeringAngle = MAX_STEERING_RIGHT;
        m_pedalLogic = 1;
        if (*m_pLeftDistance < AVOID_ACT_LEFTRIGHT) {
            m_countdown = AVOID_TURNTIME_LEFTRIGHT;
        } else {
            m_groundSteeringAngle = MAX_STEERING_RIGHT/2;
            m_countdown -= m_dt;
        }
    }
}
