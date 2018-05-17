#include "behaviour-avoid.hpp"

void BehaviourAvoid::initialState() noexcept {
    if (*m_pLeftDistance < AVOID_ACT_LEFTRIGHT) {
        //m_countdown = AVOID_TURNTIME_LEFTRIGHT;
        setState(&BehaviourAvoid::stateSideTurnRight);
    } else if (*m_pRightDistance < AVOID_ACT_LEFTRIGHT) {
        //m_countdown = AVOID_TURNTIME_LEFTRIGHT;
        setState(&BehaviourAvoid::stateSideTurnLeft);
    } else if (*m_pFrontDistance < AVOID_ACT_FRONT) {
        //m_countdown = AVOID_TURNTIME_FRONT;
        if (previousTurnRight) { setState(&BehaviourAvoid::stateFrontTurnRight); }
        else { setState(&BehaviourAvoid::stateFrontTurnLeft); }
    }
}

void BehaviourAvoid::stateFrontTurnLeft() noexcept {
    previousTurnRight = false;
    if ((*m_pFrontDistance > AVOID_ACT_FRONT) && (m_countdown < 0)) {
        setState();
    } else {
        m_groundSteeringAngle = MAX_STEERING_LEFT;
        m_pedalLogic = 1;
        if (*m_pFrontDistance < AVOID_ACT_FRONT) { m_countdown = AVOID_TURNTIME_FRONT; }
        else { m_countdown -= m_dt; }
    }
}

void BehaviourAvoid::stateFrontTurnRight() noexcept {
    previousTurnRight = true;
    if ((*m_pFrontDistance > AVOID_ACT_FRONT) && (m_countdown < 0)) {
        setState();
    } else {
        m_groundSteeringAngle = MAX_STEERING_RIGHT;
        m_pedalLogic = 1;
        if (*m_pFrontDistance < AVOID_ACT_FRONT) { m_countdown = AVOID_TURNTIME_FRONT; }
        else { m_countdown -= m_dt; }
    }
}

void BehaviourAvoid::stateSideTurnLeft() noexcept {
    previousTurnRight = false;
    if ((*m_pRightDistance > AVOID_ACT_LEFTRIGHT) && (m_countdown < 0)) {
        setState();
    } else {
        m_groundSteeringAngle = MAX_STEERING_LEFT/2;
        m_pedalLogic = 1;
        if (*m_pRightDistance < AVOID_ACT_LEFTRIGHT) { m_countdown = AVOID_TURNTIME_LEFTRIGHT; }
        else { m_countdown -= m_dt; }
    }
}

void BehaviourAvoid::stateSideTurnRight() noexcept{
    previousTurnRight = true;
    if ((*m_pLeftDistance > AVOID_ACT_LEFTRIGHT) && (m_countdown < 0)) {
        setState();
    } else {
        m_groundSteeringAngle = MAX_STEERING_RIGHT/2;
        m_pedalLogic = 1;
        if (*m_pLeftDistance < AVOID_ACT_LEFTRIGHT) { m_countdown = AVOID_TURNTIME_LEFTRIGHT; }
        else { m_countdown -= m_dt; }
    }
}