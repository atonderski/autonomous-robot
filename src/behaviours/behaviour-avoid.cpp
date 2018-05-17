#include "behaviour-avoid.hpp"

void BehaviourAvoid::universalState() noexcept {
    if (m_randomizerCountdown >= 0) { m_randomizerCountdown -= m_dt; }
    if (m_countdown >= 0) { m_countdown -= m_dt; }
}

void BehaviourAvoid::initialState() noexcept {
    if (*m_pLeftDistance < AVOID_ACT_LEFTRIGHT) {
        setState(&BehaviourAvoid::stateSideTurnRight);
    } else if (*m_pRightDistance < AVOID_ACT_LEFTRIGHT) {
        setState(&BehaviourAvoid::stateSideTurnLeft);
    } else if (*m_pFrontDistance < AVOID_ACT_FRONT) {

        if (m_randomizerCountdown < 0) {
            if (std::rand()%2) { previousTurnRight = false; }
            else { previousTurnRight = true; }
            m_randomizerCountdown = PREVIOUS_TURN_MEMORY;
        }

        if (previousTurnRight) { setState(&BehaviourAvoid::stateFrontTurnRight); }
        else { setState(&BehaviourAvoid::stateFrontTurnLeft); }
    }
}

void BehaviourAvoid::stateFrontTurnLeft() noexcept {
    if (*m_pLeftDistance < AVOID_ACT_LEFTRIGHT) { setState(&BehaviourAvoid::stateSideTurnRight); }
    if ((*m_pFrontDistance > AVOID_ACT_FRONT) && (m_countdown < 0)) {
        setState();
    } else {
        m_groundSteeringAngle = MAX_STEERING_LEFT;
        m_pedalLogic = 1;
        if (*m_pFrontDistance < AVOID_ACT_FRONT) { m_countdown = AVOID_TURNTIME_FRONT; }
    }
}

void BehaviourAvoid::stateFrontTurnRight() noexcept {
    if (*m_pRightDistance < AVOID_ACT_LEFTRIGHT) { setState(&BehaviourAvoid::stateSideTurnLeft); }
    if ((*m_pFrontDistance > AVOID_ACT_FRONT) && (m_countdown < 0)) {
        setState();
    } else {
        m_groundSteeringAngle = MAX_STEERING_RIGHT;
        m_pedalLogic = 1;
        if (*m_pFrontDistance < AVOID_ACT_FRONT) { m_countdown = AVOID_TURNTIME_FRONT; }
    }
}

void BehaviourAvoid::stateSideTurnLeft() noexcept {
    if (*m_pLeftDistance < AVOID_ACT_LEFTRIGHT) { setState(&BehaviourAvoid::stateSideTurnRight); }
    previousTurnRight = false;
    if ((*m_pRightDistance > AVOID_ACT_LEFTRIGHT) && (m_countdown < 0)) {
        setState();
    } else {
        m_groundSteeringAngle = MAX_STEERING_LEFT/2;
        m_pedalLogic = 1;
        if (*m_pRightDistance < AVOID_ACT_LEFTRIGHT) { m_countdown = AVOID_TURNTIME_LEFTRIGHT; }
    }
}

void BehaviourAvoid::stateSideTurnRight() noexcept{
    if (*m_pRightDistance < AVOID_ACT_LEFTRIGHT) { setState(&BehaviourAvoid::stateSideTurnLeft); }
    previousTurnRight = true;
    if ((*m_pLeftDistance > AVOID_ACT_LEFTRIGHT) && (m_countdown < 0)) {
        setState();
    } else {
        m_groundSteeringAngle = MAX_STEERING_RIGHT/2;
        m_pedalLogic = 1;
        if (*m_pLeftDistance < AVOID_ACT_LEFTRIGHT) { m_countdown = AVOID_TURNTIME_LEFTRIGHT; }
    }
}